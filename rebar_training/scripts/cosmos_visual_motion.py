"""Offline image-only motion evidence for this cyan-tray simulation appearance.

Tracks visible fixture corners, never native poses or policy data. Color prior is
specific to this recording; it is not a general robot/object detector.
"""
from pathlib import Path
import argparse
import json
from cosmos_reasoner_replay import now, save, sha256

PARAMETERS={'cyan_rgb':{'r_min':100,'g_min':175,'b_min':190,'g_minus_r_min':25,'b_minus_r_min':30},
    'group_dilation_pixels':51,'min_anchor_pixels':30,'min_surviving_tracks':6,
    'forward_backward_max_error_px':1.5,'moving_excursion_px':12,'stationary_excursion_px':4,
    'camera_shift_limit_px':4,'large_scene_change_threshold':12,
    'affine_reprojection_limit_px':2.5,'affine_min_inlier_ratio':.7,
    'anchor_appearance_radius_px':6}


def track(frames, points, appearance_masks=None):
    import cv2,numpy as np
    if points is None:return np.empty((0,2)),np.empty((len(frames),0,2))
    origin=points[:,0].copy();history=np.full((len(frames),len(origin),2),np.nan,dtype=np.float32)
    history[0]=origin;ids=np.arange(len(origin));current=points
    for i in range(1,len(frames)):
        if len(ids)==0:break
        nxt,status,error=cv2.calcOpticalFlowPyrLK(frames[i-1],frames[i],current,None,
            winSize=(21,21),maxLevel=3,criteria=(cv2.TERM_CRITERIA_EPS|cv2.TERM_CRITERIA_COUNT,30,.01))
        if nxt is None:break
        back,bs,be=cv2.calcOpticalFlowPyrLK(frames[i],frames[i-1],nxt,None,
            winSize=(21,21),maxLevel=3,criteria=(cv2.TERM_CRITERIA_EPS|cv2.TERM_CRITERIA_COUNT,30,.01))
        if back is None:break
        good=(status[:,0]>0)&(bs[:,0]>0)&(error[:,0]<30)&(np.linalg.norm(back[:,0]-current[:,0],axis=1)<1.5)
        if appearance_masks is not None:
            xy=np.rint(nxt[:,0]).astype(int)
            inside=(xy[:,0]>=0)&(xy[:,0]<frames[i].shape[1])&(xy[:,1]>=0)&(xy[:,1]<frames[i].shape[0])
            visible=np.zeros(len(xy),bool)
            visible[inside]=appearance_masks[i][xy[inside,1],xy[inside,0]]>0
            good &= visible
        ids=ids[good];current=nxt[good];history[i,ids]=current[:,0]
    return origin,history


def analyze(path, output):
    import av,cv2,numpy as np
    cv2.setNumThreads(2)
    with av.open(str(path)) as c:
        assert abs(float(c.streams.video[0].average_rate)-4)<.001
        rgb=[f.to_ndarray(format='rgb24') for f in c.decode(video=0)]
    gray=[cv2.cvtColor(f,cv2.COLOR_RGB2GRAY) for f in rgb]
    f=rgb[0].astype(np.int16);red,green,blue=f[:,:,0],f[:,:,1],f[:,:,2]
    mask=((red>100)&(green>175)&(blue>190)&(green-red>25)&(blue-red>30)).astype('uint8')*255
    groups=cv2.dilate(mask,np.ones((51,51),np.uint8));count,labels,stats,centroids=cv2.connectedComponentsWithStats(groups)
    scores=[int(np.count_nonzero(mask[labels==i])) for i in range(1,count)]
    anchor=np.zeros_like(mask);bbox=None
    if scores and max(scores)>=30:
        label=1+int(np.argmax(scores));anchor[labels==label]=mask[labels==label]
        anchor=cv2.dilate(anchor,np.ones((5,5),np.uint8));bbox=stats[label,:4].tolist()
    p=cv2.goodFeaturesToTrack(gray[0],maxCorners=120,qualityLevel=.005,minDistance=3,mask=anchor,blockSize=3)
    background=np.ones_like(mask)*255
    if bbox:
        x,y,w,h=bbox;background[max(0,y-30):y+h+30,max(0,x-30):x+w+30]=0
    b=cv2.goodFeaturesToTrack(gray[0],maxCorners=300,qualityLevel=.01,minDistance=8,mask=background,blockSize=5)
    appearance=[]
    for frame in rgb:
        values=frame.astype(np.int16);rr,gg,bb=values[:,:,0],values[:,:,1],values[:,:,2]
        visible=((rr>100)&(gg>175)&(bb>190)&(gg-rr>25)&(bb-rr>30)).astype('uint8')*255
        appearance.append(cv2.dilate(visible,np.ones((13,13),np.uint8)))
    origin,hist=track(gray,p,appearance);bg,bhist=track(gray,b)
    # Median displacement of surviving scene corners rejects viewpoint changes.
    bg_shift=[]
    for i in range(len(gray)):
        valid=np.isfinite(bhist[i,:,0])
        bg_shift.append(np.median(bhist[i,valid]-bg[valid],axis=0) if valid.sum()>=10 else np.array([np.nan,np.nan]))
    bg_shift=np.array(bg_shift)
    frame_changes=[float(np.mean(np.abs(gray[i].astype(float)-gray[i-1]))) for i in range(1,len(gray))]
    camera_changed=bool(np.any(np.linalg.norm(bg_shift,axis=1)>4))
    if max(frame_changes,default=0)>12 and np.isfinite(bhist[-1,:,0]).sum()<max(10,len(bg)*.4):camera_changed=True
    survive=np.isfinite(hist[-1,:,0]);selected=hist[:,survive];initial=origin[survive]
    result={'at':now(),'input_sha256':sha256(path),'parameters':PARAMETERS,'analysis_script_sha256':sha256(Path(__file__)),
        'method':'Cyan fixture grouping + pyramidal LK with forward/backward check + scene registration',
        'native_pose_or_labels_used':False,'appearance_specific_prior':True,'camera_changed_or_unstable':camera_changed,
        'anchor_bbox_first_frame':bbox,'initial_anchor_tracks':len(origin),'surviving_anchor_tracks':int(survive.sum()),
        'background_initial_tracks':len(bg),'background_surviving_tracks':int(np.isfinite(bhist[-1,:,0]).sum()),
        'max_adjacent_mean_image_change':max(frame_changes,default=0),'decision':'uncertain',
        'measurement_units':'pixels only, not metres or physical robot pose','physical_acceptance':False}
    # A verified movement before later occlusion is still observed movement.
    # Stationary requires surviving anchors at the end; lost tracks cannot prove it.
    observations=[]
    if not camera_changed:
        for index in range(len(gray)):
            valid=np.isfinite(hist[index,:,0])
            if valid.sum()<6 or not np.all(np.isfinite(bg_shift[index])):continue
            initial_at_frame=origin[valid];end_at_frame=hist[index,valid]-bg_shift[index]
            delta=end_at_frame-initial_at_frame;vector=np.median(delta,axis=0)
            excursion=float(np.median(np.linalg.norm(delta,axis=1)))
            spread=float(np.median(np.linalg.norm(delta-vector,axis=1)))
            affine,inliers=cv2.estimateAffine2D(initial_at_frame,end_at_frame,
                method=cv2.RANSAC,ransacReprojThreshold=2.5,maxIters=2000,confidence=.99)
            ratio=float(np.mean(inliers)) if inliers is not None else 0
            if spread<=max(8,excursion*.5) or ratio>=.7:
                observations.append((excursion,index,int(valid.sum()),vector,spread,ratio,affine))
    if observations:
        excursion,index,n,vector,spread,ratio,affine=max(observations,key=lambda value:value[0])
        result.update(max_median_excursion_px=excursion,observed_peak_relative_s=index/4,
            observed_peak_track_count=n,observed_peak_displacement_xy_px=vector.tolist(),
            observed_peak_vector_spread_px=spread,affine_inlier_ratio=ratio,
            end_anchor_visible=bool(survive.sum()>=6),
            excursion_definition='max over valid frames of median point displacement after scene registration')
        if affine is not None:result['image_affine_first_to_peak']=affine.tolist()
        if excursion>12:result['decision']='moving'
        elif excursion<4 and survive.sum()>=6:result['decision']='stationary'
        peak_valid=np.isfinite(hist[index,:,0])
        result['observed_tracks']=[{'start':a.tolist(),'observed_end':z.tolist(),
            'relative_s':index/4} for a,z in zip(origin[peak_valid],hist[index,peak_valid])]
        if survive.sum()>=6 and np.all(np.isfinite(bg_shift[-1])):
            result['median_end_displacement_xy_px']=np.median(selected[-1]-initial-bg_shift[-1],axis=0).tolist()
            result['background_end_shift_xy_px']=bg_shift[-1].tolist()
    else:result['end_anchor_visible']=False
    canvas=cv2.cvtColor(rgb[0],cv2.COLOR_RGB2BGR)
    if bbox:
        x,y,w,h=bbox;cv2.rectangle(canvas,(x,y),(x+w,y+h),(0,180,255),2)
    for item in result.get('observed_tracks',[]):
        cv2.arrowedLine(canvas,tuple(np.rint(item['start']).astype(int)),
                       tuple(np.rint(item['observed_end']).astype(int)),(0,0,255),1,tipLength=.15)
    output.parent.mkdir(parents=True,exist_ok=True);cv2.imwrite(str(output.with_suffix('.png')),canvas)
    save(output,result);return result


def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--output',type=Path,required=True);p.add_argument('--only',type=int,nargs='+')
    args=p.parse_args();r=args.output.resolve();m=json.loads((r/'manifest.json').read_text())
    for clip in m['clips']:
        if clip['view']!='overview' or args.only and clip['index'] not in args.only:continue
        path=r/clip['path'];assert sha256(path)==clip['sha256']
        result=analyze(path,r/'visual_motion'/(clip['id']+'.json'))
        print(json.dumps({k:result[k] for k in ['decision','initial_anchor_tracks','surviving_anchor_tracks','camera_changed_or_unstable']}|{'id':clip['id'],'excursion_px':result.get('max_median_excursion_px')}),flush=True)


if __name__=='__main__':main()
