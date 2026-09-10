import QtQuick

Item {
    id: gizmo
    objectName: "endEffectorGizmo"
    required property var view3d
    property vector3d center
    property var basis
    property var orientation
    property var segments: []
    signal dragged(int axis, double amount, bool rotation)
    function point(offset) {
        return view3d.mapFrom3DScene(Qt.vector3d(center.x+offset[0],center.y+offset[1],center.z+offset[2]))
    }
    function segmentDistance(x,y,a,b) {
        let dx=b.x-a.x, dy=b.y-a.y
        let t=Math.max(0,Math.min(1,((x-a.x)*dx+(y-a.y)*dy)/(dx*dx+dy*dy || 1)))
        return Math.hypot(x-a.x-t*dx,y-a.y-t*dy)
    }
    Canvas {
        id: canvas; anchors.fill: parent
        onPaint: {
            let ctx=getContext("2d"); ctx.reset()
            let c=gizmo.point([0,0,0]); let parts=[]
            if (c.z <= 0) { gizmo.segments=[]; return }
            let colors=["#ff665c","#60e78a","#58a7ff"]
            for (let axis=0;axis<3;axis++) {
                let v=gizmo.basis[axis]
                let tip=gizmo.point(v.map(x=>x*22))
                ctx.strokeStyle=colors[axis]; ctx.fillStyle=colors[axis];ctx.lineWidth=4
                ctx.beginPath();ctx.moveTo(c.x,c.y);ctx.lineTo(tip.x,tip.y);ctx.stroke()
                ctx.beginPath();ctx.arc(tip.x,tip.y,6,0,2*Math.PI);ctx.fill()
                ctx.font="bold 16px sans-serif";ctx.fillText(["X","Y","Z"][axis],tip.x+8,tip.y-8)
                parts.push({axis:axis,rotation:false,a:c,b:tip})
                let u=gizmo.basis[(axis+1)%3],w=gizmo.basis[(axis+2)%3]
                ctx.lineWidth=2;ctx.globalAlpha=.8
                let previous=null
                for(let i=0;i<=72;i++) {
                    let angle=i*2*Math.PI/72
                    let p=gizmo.point(u.map((v,j)=>(v*Math.cos(angle)+w[j]*Math.sin(angle))*14))
                    if(previous) {
                        ctx.beginPath();ctx.moveTo(previous.x,previous.y);ctx.lineTo(p.x,p.y);ctx.stroke()
                        parts.push({axis:axis,rotation:true,a:previous,b:p})
                    }
                    previous=p
                }
                ctx.globalAlpha=1
                // Thin target-orientation axes distinguish rotation from the fixed-frame handles.
                let o=gizmo.point(gizmo.orientation[axis].map(x=>x*9))
                ctx.lineWidth=1;ctx.beginPath();ctx.moveTo(c.x,c.y);ctx.lineTo(o.x,o.y);ctx.stroke()
            }
            gizmo.segments=parts
        }
    }
    Timer { interval: 33; running: gizmo.visible; repeat: true; onTriggered: canvas.requestPaint() }
    MouseArea {
        anchors.fill: parent
        property var picked: null
        property real oldX: 0; property real oldY: 0
        onPressed: function(mouse) {
            let best=10, found=null
            for(let s of gizmo.segments) {
                let distance=gizmo.segmentDistance(mouse.x,mouse.y,s.a,s.b)
                if(distance<best) { best=distance;found=s }
            }
            if(!found) { mouse.accepted=false; return }
            picked=found;oldX=mouse.x;oldY=mouse.y
        }
        onPositionChanged: function(mouse) {
            if(!pressed || !picked) return
            let dx=mouse.x-oldX,dy=mouse.y-oldY
            if(picked.rotation) {
                // Find the nearest current projected ring tangent for this rotation axis.
                let nearest=picked, best=1e9
                for(let s of gizmo.segments) if(s.rotation && s.axis===picked.axis) {
                    let d=gizmo.segmentDistance(oldX,oldY,s.a,s.b)
                    if(d<best) {best=d;nearest=s}
                }
                let tx=nearest.b.x-nearest.a.x,ty=nearest.b.y-nearest.a.y
                let length=Math.hypot(tx,ty)
                if(length>1) gizmo.dragged(picked.axis,Math.max(-.15,Math.min(.15,(dx*tx+dy*ty)/(length*length)*2*Math.PI/72)),true)
            } else {
                let c=gizmo.point([0,0,0]),v=gizmo.basis[picked.axis],tip=gizmo.point(v.map(x=>x*22))
                let ax=tip.x-c.x,ay=tip.y-c.y,l=ax*ax+ay*ay
                if(l>100) gizmo.dragged(picked.axis,Math.max(-.03,Math.min(.03,(dx*ax+dy*ay)/l*.22)),false)
            }
            oldX=mouse.x;oldY=mouse.y
        }
        onReleased: picked=null
        onCanceled: picked=null
        onWheel: function(wheel) { wheel.accepted=false }
    }
}
