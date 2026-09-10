"""Persistent named joint positions; saved values are the executed targets."""
import json
import math
from pathlib import Path
PRESET_NAMES = ('放置位', '存储位 1', '存储位 2', '存储位 3', '存储位 4')

class PresetStore:
    def __init__(self,path):
        self.path=Path(path);self.values={};self.load()
    @staticmethod
    def validate(values):
        if not isinstance(values,(list,tuple)) or len(values)!=6:
            raise ValueError('快捷位必须包含六个关节角')
        values=[float(v) for v in values]
        if not all(math.isfinite(v) and -360<=v<=360 for v in values):
            raise ValueError('快捷位角度无效或超出限位')
        return values
    def load(self):
        try:
            raw=json.loads(self.path.read_text(encoding='utf-8'))
            if not isinstance(raw,dict):return
            for name in PRESET_NAMES:
                if name in raw:
                    try:self.values[name]=self.validate(raw[name])
                    except (ValueError,TypeError):pass
        except (OSError,ValueError,TypeError):pass
    def set(self,name,joints_deg):
        if name not in PRESET_NAMES:raise ValueError('无效快捷位名称')
        values=self.validate(joints_deg)
        updated=dict(self.values);updated[name]=[round(v,6) for v in values]
        self.path.parent.mkdir(parents=True,exist_ok=True)
        temporary=self.path.with_suffix('.tmp')
        temporary.write_text(json.dumps(updated,ensure_ascii=False,indent=2,allow_nan=False),encoding='utf-8')
        temporary.replace(self.path);self.values=updated
    def get(self,name):
        value=self.values.get(name)
        return list(value) if value is not None else None
