// (c) Facebook, Inc. and its affiliates.

//#region Modules
const S = require('Scene');
const BT = require('BodyTracking');
const CI = require('CameraInfo');
const R = require('Reactive');
////#endregion

//#region Global Variables
const width = CI.previewSize.width.div(CI.previewScreenScale);
const height = CI.previewSize.height.div(CI.previewScreenScale);

const Config = {
    camera_size: {
        width:  width,
        height: height},
    safeArea: {
        minX : width.mul(0.05),
        maxX : width.mul(0.95),
        minY : height.mul(0.05),
        maxY : height.mul(0.95)},
    jnt_smoothing: 50,
    body_scale: R.vector(600, 600, 1),
    feet_multiplier: 0.35,
    hand_multiplier: 0.7
}

// BODY
const Body = BT.body(0);
const Pose = Body.pose2D;
const Head = Pose.head;
const LArm = Pose.leftArm;
const RArm = Pose.rightArm;
const Torso = Pose.torso;
const LLeg = Pose.leftLeg;
const RLeg = Pose.rightLeg;
const headDist = getScale(Head.topHead,  Head.chin).mul(Config.camera_size.height.div(Config.camera_size.width)).mul(0.88);
const headScale = R.point(headDist, headDist, 1.0);
const chinPosYLow = Head.chin.keyPoint.y.mul(Config.camera_size.height).gt(Config.camera_size.height.mul(0.7));
const isCloseup = headScale.x.div(2.5).gt(Config.body_scale.x).and(chinPosYLow);
const hideMeshes = isCloseup.or(BT.count.lt(1.0));

function getAngleBetweenPoints(pointA, pointB){

    const pAx = pointA.keyPoint.x.mul(Config.camera_size.width).expSmooth(Config.jnt_smoothing);
    const pAy = pointA.keyPoint.y.mul(Config.camera_size.height).expSmooth(Config.jnt_smoothing).mul(-1.0);

    const pBx = pointB.keyPoint.x.mul(Config.camera_size.width).expSmooth(Config.jnt_smoothing);
    const pBy = pointB.keyPoint.y.mul(Config.camera_size.height).expSmooth(Config.jnt_smoothing).mul(-1.0);

    const y = pBy.sub(pAy);
    const x = pBx.sub(pAx);
    const angle = R.atan2( y, x);
    return angle;
}

function getAveragePosition(pointA, pointB){
    const pAx = pointA.keyPoint.x.mul(Config.camera_size.width).expSmooth(Config.jnt_smoothing);
    const pAy = pointA.keyPoint.y.mul(Config.camera_size.height).expSmooth(Config.jnt_smoothing).mul(-1.0);

    const pBx = pointB.keyPoint.x.mul(Config.camera_size.width).expSmooth(Config.jnt_smoothing);
    const pBy = pointB.keyPoint.y.mul(Config.camera_size.height).expSmooth(Config.jnt_smoothing).mul(-1.0);

    const averagePosX = pAx.add(pBx).div(2.0);
    const averagePosY = pAy.add(pBy).div(2.0);
    return R.point2d(averagePosX, averagePosY);
}

function getScale(pointA, pointB){
    var multiplier =  R.distance(pointA.keyPoint, pointB.keyPoint).div(62).expSmooth(Config.jnt_smoothing);
    return multiplier.mul(Config.body_scale.x).mul(Config.camera_size.width);
}

function offsetPoint(point_a, point_b, multiplier){
    const x = point_b.x.sub(point_a.x).mul(multiplier).add(point_b.x);
    const y = point_b.y.sub(point_a.y).mul(multiplier).add(point_b.y);

    return R.point2d(x, y);
}

function setJointTransform(joint, mainKp, angle, scale, mesh, torsoDep){
    const parentX = mainKp.keyPoint.x.mul(Config.camera_size.width).expSmooth(Config.jnt_smoothing);
    const parentY = mainKp.keyPoint.y.mul(Config.camera_size.height).expSmooth(Config.jnt_smoothing).mul(-1.0);

    joint.transform.x = parentX;
    joint.transform.y = parentY;
    if(scale == null){
        joint.transform.scale = Config.body_scale;
    }
    else{
        joint.transform.scale = scale;
    }
    if(angle != null){
        joint.transform.rotationZ = getAngleBetweenPoints(angle[0], angle[1]);
    }
    if(mesh != null){
        const visibleY = parentY.mul(-1.0).gt(Config.safeArea.minY).and(parentY.mul(-1.0).lt(Config.safeArea.maxY));
        const visibleX = parentX.gt(Config.safeArea.minY).and(parentX.lt(Config.safeArea.maxX));
        var hideMesh;
        if(torsoDep == null){
            hideMesh = visibleY.and(visibleX).not().or(hideMeshes);
        }
        else{
            hideMesh = visibleY.and(visibleX).not().or(hideMeshes).or(torsoDep);
        }
        mesh.hidden = hideMesh;
    }
}
////#endregion

Promise.all([
    S.root.findFirst('rigNull'),
    S.root.findByPath('**/bodyRig/Armature/skeleton/*'),
    S.root.findByPath('**/bodyRig/bodyNull/*'),
    ])
    .then(results =>{

        const rigNull =       results[0];
        const rig =           results[1];
        const meshes =        results[2];
        //joints
        const Chin =                      rig[0];
        const RShoulder =                 rig[1];
        const RElbowShoulder =            rig[2];
        const RElbowWrist =               rig[3];
        const RWrist =                    rig[4];
        const RHand =                     rig[5];
        const LShoulder =                 rig[6];
        const LElbowShoulder =            rig[7];
        const LElbowWrist =               rig[8];
        const LWrist =                    rig[9];
        const LHand =                     rig[10];
        const RTorso =                    rig[11];
        const RHipTorso =                 rig[12];
        const RHip =                      rig[13];
        const RKneeHip =                  rig[14];
        const RKneeAnkle =                rig[15];
        const RAnkle =                    rig[16];
        const RFoot =                     rig[17];
        const LTorso =                    rig[18];
        const LHipTorso =                 rig[19];
        const LHip =                      rig[20];
        const LKneeHip =                  rig[21];
        const LKneeAnkle =                rig[22];
        const LAnkle =                    rig[23];
        const LFoot =                     rig[24];
        //meshes
        const headMesh =                 meshes[0];
        const glassesMesh =              meshes[1];
        const torsoMesh =                meshes[2];
        const rArmMesh =                 meshes[3];
        const rForeArmMesh =             meshes[4];
        const rHandMesh =                meshes[5];
        const lArmMesh =                 meshes[6];
        const lForeArmMesh =             meshes[7];
        const lHandMesh =                meshes[8];
        const rThighMesh =               meshes[9];
        const rLegMesh =                 meshes[10];
        const rFootMesh =                meshes[11];
        const lThighMesh =               meshes[12];
        const lLegMesh =                 meshes[13];
        const lFootMesh =                meshes[14];

        //Pin Rig to top corner
        rigNull.transform.x = Config.camera_size.width.div(-2.0);
        rigNull.transform.y = Config.camera_size.height.div(2.0);
        rigNull.transform.scaleX = rigNull.transform.scaleY = rigNull.transform.scaleZ = 1;

        // Head & Glasses Transform
        const averageEyes = getAveragePosition(Head.leftEye, Head.rightEye);
        glassesMesh.transform.x = averageEyes.x;
        glassesMesh.transform.y = averageEyes.y;

        Chin.transform.x = Head.chin.keyPoint.x.mul(Config.camera_size.width).expSmooth(Config.jnt_smoothing);
        Chin.transform.y = Head.chin.keyPoint.y.mul(Config.camera_size.height).expSmooth(Config.jnt_smoothing).mul(-1.0);

         // Scale
        glassesMesh.transform.scale = Chin.transform.scale = headScale;

         // Rotation
        const lEyeY = Head.leftEye.keyPoint.y.mul(Config.camera_size.height).expSmooth(Config.jnt_smoothing).mul(-1.0);
        const rEyeY = Head.rightEye.keyPoint.y.mul(Config.camera_size.height).expSmooth(Config.jnt_smoothing).mul(-1.0);
        const rotZ = lEyeY.sub(rEyeY).mul(3.1416/180.0).mul(0.85);
        Chin.transform.rotationZ = glassesMesh.transform.rotationZ = rotZ;

         //visibility
        const visibleY = Chin.transform.y.mul(-1.0).gt(Config.safeArea.minY).and(Chin.transform.y.mul(-1.0).lt(Config.safeArea.maxY));
        const visibleX = Chin.transform.x.gt(Config.safeArea.minY).and(Chin.transform.x.lt(Config.safeArea.maxX));
        glassesMesh.hidden = headMesh.hidden = visibleY.and(visibleX).not().or(BT.count.lt(1));

        torsoMesh.hidden = lArmMesh.hidden.and(rArmMesh.hidden);

        //Calculate Scale
        const LArmScale = R.point(Config.body_scale.x, getScale(LArm.shoulder, LArm.elbow), 1.0);
        const RArmScale = R.point(Config.body_scale.x, getScale(RArm.shoulder, RArm.elbow), 1.0);
        const LLegScale = R.point(getScale(Torso.leftHip, LLeg.knee), Config.body_scale.x,  1.0);
        const RLegScale = R.point(getScale(Torso.rightHip, RLeg.knee), Config.body_scale.x, 1.0);

        const kp_joints = [
            { KP: LArm.shoulder,   joint: LTorso,           angle: null                        , scale: LArmScale, mesh: null         , tDep: null             },
            { KP: LArm.shoulder,   joint: LShoulder ,       angle: [LArm.shoulder, LArm.elbow] , scale: LArmScale, mesh: lArmMesh     , tDep: null             },
            { KP: LArm.elbow,      joint: LElbowShoulder,   angle: [LArm.shoulder, LArm.elbow] , scale: LArmScale, mesh: null         , tDep: null             },
            { KP: LArm.elbow,      joint: LElbowWrist,      angle: [LArm.elbow,    LArm.wrist] , scale: LArmScale, mesh: lForeArmMesh , tDep: null             },
            { KP: LArm.wrist,      joint: LWrist,           angle: [LArm.elbow,    LArm.wrist] , scale: LArmScale, mesh: lHandMesh    , tDep: null             },
            { KP: LLeg.knee,       joint: LKneeHip,         angle: null                        , scale: LLegScale, mesh: null         , tDep: null             },
            { KP: LLeg.knee,       joint: LKneeAnkle,       angle: null                        , scale: LLegScale, mesh: lLegMesh     , tDep: torsoMesh.hidden },
            { KP: LLeg.ankle,      joint: LAnkle,           angle: null                        , scale: LLegScale, mesh: lFootMesh    , tDep: torsoMesh.hidden },
            { KP: Torso.leftHip,   joint: LHipTorso,        angle: null                        , scale: LLegScale, mesh: null         , tDep: null             },
            { KP: Torso.leftHip,   joint: LHip,             angle: null                        , scale: LLegScale, mesh: lThighMesh   , tDep: torsoMesh.hidden },

            { KP: RArm.shoulder,   joint: RTorso,           angle: null                        , scale: RArmScale, mesh: null         , tDep: null             },
            { KP: RArm.shoulder,   joint: RShoulder ,       angle: [RArm.elbow, RArm.shoulder] , scale: RArmScale, mesh: rArmMesh     , tDep: null             },
            { KP: RArm.elbow,      joint: RElbowShoulder,   angle: [RArm.elbow, RArm.shoulder] , scale: RArmScale, mesh: null         , tDep: null             },
            { KP: RArm.elbow,      joint: RElbowWrist,      angle: [RArm.wrist,    RArm.elbow] , scale: RArmScale, mesh: rForeArmMesh , tDep: null             },
            { KP: RArm.wrist,      joint: RWrist,           angle: [RArm.wrist,    RArm.elbow] , scale: RArmScale, mesh: rHandMesh    , tDep: null             },
            { KP: RLeg.knee,       joint: RKneeHip,         angle: null                        , scale: RLegScale, mesh: null         , tDep: null             },
            { KP: RLeg.knee,       joint: RKneeAnkle,       angle: null                        , scale: RLegScale, mesh: rLegMesh     , tDep: torsoMesh.hidden },
            { KP: RLeg.ankle,      joint: RAnkle,           angle: null                        , scale: RLegScale, mesh: rFootMesh    , tDep: torsoMesh.hidden },
            { KP: Torso.rightHip,  joint: RHipTorso,        angle: null                        , scale: RLegScale, mesh: null         , tDep: null             },
            { KP: Torso.rightHip,  joint: RHip,             angle: null                        , scale: RLegScale, mesh: rThighMesh   , tDep: null             }
        ]

        const limbs = [
            { joint: LHand, pointA: LElbowWrist, pointB: LWrist, scale: Config.hand_multiplier },
            { joint: RHand, pointA: RElbowWrist, pointB: RWrist, scale: Config.hand_multiplier },
            { joint: LFoot, pointA: LKneeAnkle,  pointB: LAnkle, scale: Config.feet_multiplier },
            { joint: RFoot, pointA: RKneeAnkle,  pointB: RAnkle, scale: Config.feet_multiplier },
        ]

        setRig(kp_joints, limbs);

})

function setRig(rig_array, limb_array){

    rig_array.forEach(e => {
        setJointTransform(e.joint, e.KP, e.angle, e.scale, e.mesh, e.tDep);
    });

    limb_array.forEach(limb => {
        var pos = offsetPoint(limb.pointA.transform, limb.pointB.transform, limb.scale);
        limb.joint.transform.x = pos.x;
        limb.joint.transform.y = pos.y;
        limb.joint.transform.scale = Config.body_scale;
        limb.joint.transform.rotation = limb.pointB.transform.rotation;
    });
}
