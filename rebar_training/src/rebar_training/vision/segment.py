"""Small grayscale steel segmenter; runtime inputs are images only.

Isaac labels are read exclusively by the training dataset. Validation is split
by independent capture roots/episodes, never random neighbouring video frames.
"""
import argparse
import contextlib
import json
import random
from pathlib import Path

import numpy as np
from PIL import Image
import torch
from torch import nn
from torch.nn import functional as F


class Block(nn.Sequential):
    def __init__(self, inputs, outputs):
        super().__init__(nn.Conv2d(inputs, outputs, 3, padding=1), nn.GroupNorm(4, outputs), nn.SiLU(),
                         nn.Conv2d(outputs, outputs, 3, padding=1), nn.GroupNorm(4, outputs), nn.SiLU())


class SteelSegmenter(nn.Module):
    def __init__(self):
        super().__init__()
        self.enc1, self.enc2, self.enc3 = Block(1, 12), Block(12, 24), Block(24, 48)
        self.bottom = Block(48, 72)
        self.dec3, self.dec2, self.dec1 = Block(120, 48), Block(72, 24), Block(36, 12)
        self.output = nn.Conv2d(12, 1, 1)

    def forward(self, x):
        a = self.enc1(x); b = self.enc2(F.max_pool2d(a, 2)); c = self.enc3(F.max_pool2d(b, 2))
        d = self.bottom(F.max_pool2d(c, 2))
        d = self.dec3(torch.cat((F.interpolate(d, size=c.shape[-2:], mode='bilinear', align_corners=False), c), 1))
        d = self.dec2(torch.cat((F.interpolate(d, size=b.shape[-2:], mode='bilinear', align_corners=False), b), 1))
        return self.output(self.dec1(torch.cat((F.interpolate(d, size=a.shape[-2:], mode='bilinear', align_corners=False), a), 1)))


def records(roots):
    rows, episodes = [], set()
    for root in roots:
        root = Path(root)
        meta = json.loads((root/'dataset.json').read_text())
        if meta.get('fixture') or meta['status'] != 'closed':
            raise ValueError('Use closed, real rendered capture datasets')
        for episode in (root/'episodes').glob('*/frames.jsonl'):
            episodes.add(episode.parent.name)
            for line in episode.read_text().splitlines():
                row = json.loads(line)
                camera = row['cameras']['wrist']
                labels = camera['semantic_mapping']['idToLabels']
                ids = [int(k) for k, value in labels.items() if value['class'] == 'rebar']
                rows.append((root/camera['image'], root/camera['semantic_ids'], ids, episode.parent.name))
    if not rows:
        raise ValueError('No labelled wrist frames')
    return rows, episodes


def tensors(rows, size):
    images, masks = [], []
    for image_path, labels_path, ids, _ in rows:
        image = Image.open(image_path).convert('L').resize(size, Image.Resampling.BILINEAR)
        mask = Image.fromarray(np.isin(np.load(labels_path, allow_pickle=False), ids).astype(np.uint8)).resize(size, Image.Resampling.NEAREST)
        images.append(np.asarray(image, dtype=np.float32)[None]/255.)
        masks.append(np.asarray(mask, dtype=np.float32)[None])
    return torch.from_numpy(np.stack(images)), torch.from_numpy(np.stack(masks))


class ImageDetector:
    def __init__(self, checkpoint, device='cpu'):
        saved = torch.load(checkpoint, map_location='cpu', weights_only=True)
        if saved['architecture'] != 'steel_gray_unet_v1':
            raise ValueError('Unsupported image detector')
        self.size = tuple(saved['input_size'])
        self.device = device
        self.model = SteelSegmenter().to(device).eval()
        self.model.load_state_dict(saved['model'])

    @torch.inference_mode()
    def probabilities(self, image):
        image = np.asarray(image)
        if image.dtype != np.uint8 or image.ndim != 2:
            raise ValueError('Expected mono8 image, with no depth or semantic labels')
        x = torch.from_numpy(image.copy()).to(self.device).float()[None, None]/255.
        x = F.interpolate(x, size=self.size[::-1], mode='bilinear', align_corners=False)
        logits = self.model(x)
        return F.interpolate(logits.sigmoid(), size=image.shape, mode='bilinear', align_corners=False)[0, 0].cpu().numpy()


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--train', type=Path, nargs='+', required=True)
    p.add_argument('--validation', type=Path, nargs='+', required=True)
    p.add_argument('--output', type=Path, required=True)
    p.add_argument('--epochs', type=int, default=40)
    p.add_argument('--batch-size', type=int, default=16)
    p.add_argument('--seed', type=int, default=1242)
    p.add_argument('--device', default='cuda:0')
    args = p.parse_args()
    if args.output.exists():
        p.error('Use a new checkpoint path')
    random.seed(args.seed); np.random.seed(args.seed); torch.manual_seed(args.seed)
    torch.set_num_threads(4)
    train_rows, train_ids = records(args.train); val_rows, val_ids = records(args.validation)
    if train_ids & val_ids or set(map(Path.resolve, args.train)) & set(map(Path.resolve, args.validation)):
        raise ValueError('Training and validation episodes must be independent')
    size = (320, 192)
    x, y = tensors(train_rows, size); vx, vy = tensors(val_rows, size)
    x, y, vx, vy = [t.to(args.device) for t in (x, y, vx, vy)]
    model = SteelSegmenter().to(args.device)
    optimizer = torch.optim.AdamW(model.parameters(), lr=1e-3, weight_decay=1e-4)
    scheduler = torch.optim.lr_scheduler.CosineAnnealingLR(optimizer, args.epochs, eta_min=3e-5)
    best, history = -1., []
    for epoch in range(args.epochs):
        model.train()
        for ids in torch.randperm(len(x), device=args.device).split(args.batch_size):
            image, target = x[ids], y[ids]
            # Identical flips/translations for pixels and labels; appearance
            # augmentation affects pixels only and never invents geometry.
            if random.random() < .5:
                image, target = image.flip(-1), target.flip(-1)
            if random.random() < .5:
                image, target = image.flip(-2), target.flip(-2)
            shift = (random.randint(-20, 20), random.randint(-30, 30))
            image, target = image.roll(shift, (-2, -1)), target.roll(shift, (-2, -1))
            image = (image * random.uniform(.65, 1.35) + random.uniform(-.12, .12)
                     + torch.randn_like(image)*random.uniform(0, .02)).clamp(0, 1)
            amp = torch.autocast(device_type='cuda', dtype=torch.bfloat16) if str(args.device).startswith('cuda') else contextlib.nullcontext()
            with amp:
                logits = model(image)
                probability = logits.sigmoid()
                dice = 1 - ((2*(probability*target).sum((-2,-1))+1)/(probability.sum((-2,-1))+target.sum((-2,-1))+1)).mean()
                loss = F.binary_cross_entropy_with_logits(logits, target, pos_weight=logits.new_tensor(3.)) + dice
            optimizer.zero_grad(); loss.backward(); optimizer.step()
        scheduler.step()
        model.eval()
        with torch.no_grad():
            intersection, union, n = 0., 0., 0
            for bx, by in zip(vx.split(args.batch_size), vy.split(args.batch_size)):
                pred = model(bx).sigmoid() > .5
                intersection += (pred & by.bool()).sum().item()
                union += (pred | by.bool()).sum().item()
                n += len(bx)
            iou = intersection/max(union, 1)
        record = dict(epoch=epoch, heldout_pixel_iou=iou)
        history.append(record)
        if iou > best:
            best = iou
            args.output.parent.mkdir(parents=True, exist_ok=True)
            torch.save(dict(architecture='steel_gray_unet_v1', input_size=list(size),
                            model={k:v.cpu() for k,v in model.state_dict().items()},
                            train_episodes=sorted(train_ids), validation_episodes=sorted(val_ids),
                            best_iou=best, input_contract='mono8_pixels_only'), args.output)
        if epoch % 5 == 0 or epoch == args.epochs-1:
            print(record, flush=True)
    args.output.with_suffix('.json').write_text(json.dumps(dict(train_frames=len(x), validation_frames=len(vx),
        split='independent_capture_episodes', best_heldout_pixel_iou=best, history=history,
        camera_pose_or_steel_truth_input=False, closed_loop_policy_validated=False), indent=2)+'\n')


if __name__ == '__main__':
    main()
