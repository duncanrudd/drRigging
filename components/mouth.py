import pymel.core as pmc
import drRigging.utils.coreUtils as coreUtils
import drRigging.objects.controls as drControls
import drRigging.utils.curveUtils as curveUtils

reload(coreUtils)

crv = pmc.PyNode('mouth_M_lips_rail_crv')
jawCrv = pmc.PyNode('mouth_M_jaw_rail_crv')
scaleAttr = pmc.Attribute('mouth_M_startMtx2Srt_utl.outputScaleX')
numJoints = 9

#---------------------
# curve info
#---------------------
crv_inf = pmc.createNode('curveInfo', name='mouth_M_railInfo_utl')
crv.worldSpace[0].connect(crv_inf.inputCurve)
crv_length = coreUtils.multiplyDouble(crv_inf.arcLength, scaleAttr, name='mouth_M_railLength_utl')

ctrlSize = crv_length.output.get()*.025

def makeCtrl(name, invert=0, minParam=0, maxParam=1, startParam=0.25):

    ctrl = drControls.ballCtrl(radius=ctrlSize)
    ctrl.rename('mouth_M_%s_ctrl' % name)

    # buffer
    buffer = coreUtils.addParent(ctrl, 'group', name=ctrl.name().replace('ctrl', 'ctrl_zeroSrt'))

    # negate translate x
    negX = coreUtils.addParent(ctrl, 'group', name=ctrl.name().replace('ctrl', 'neg_tx_srt'))
    uc = coreUtils.convert(ctrl.tx, -1, name='mouth_M_%s_tx_inverse_utl' % name)
    uc.output.connect(negX.tx)

    # attach to crv
    mp = pmc.createNode('motionPath', name='mouth_M_%s_mp_utl' % name)
    mp.fractionMode.set(1)
    crv.worldSpace[0].connect(mp.geometryPath)
    mp.follow.set(1)
    mp.frontAxis.set(0)
    mp.upAxis.set(1)
    mp.worldUpType.set(2)
    crv.worldMatrix[0].connect(mp.worldUpMatrix)

    mp2 = pmc.createNode('motionPath', name='mouth_M_%s_mp_utl' % name)
    mp2.fractionMode.set(1)
    jawCrv.worldSpace[0].connect(mp2.geometryPath)
    mp2.follow.set(1)
    mp2.frontAxis.set(0)
    mp2.upAxis.set(1)
    mp2.worldUpType.set(2)
    jawCrv.worldMatrix[0].connect(mp2.worldUpMatrix)

    pb = coreUtils.pairBlend(mp.allCoordinates, mp.rotate, mp2.allCoordinates, mp2.rotate, name='mouth_M_%s_jawBlend_utl' % name)
    pb.outTranslate.connect(buffer.t)
    pb.outRotate.connect(buffer.r)

    driver = ctrl
    if invert:
        driver = negX
        buffer.sx.set(-1)
    tx2Param = coreUtils.divide(driver.tx, crv_length.output, name='mouth_M_%sParam_utl' % name)
    defaultParam = pmc.createNode('animBlendNodeAdditive', name='mouth_M_%sDefaultParam_utl' % name)
    if type(minParam) == pmc.general.Attribute:
        minParam.connect(defaultParam.inputA)
    else:
        defaultParam.inputA.set(minParam)

    if type(maxParam) == pmc.general.Attribute:
        maxParam.connect(defaultParam.inputB)
    else:
        defaultParam.inputB.set(maxParam)
    defaultParam.weightA.set(1.0 - startParam)
    defaultParam.weightB.set(startParam)

    paramOffset = coreUtils.add([defaultParam.output, tx2Param.outputX], name='mouth_M_%sParamSum_utl' % name)
    paramOffset.output1D.connect(mp.uValue)
    paramOffset.output1D.connect(mp2.uValue)

    mtx2Srt = pmc.createNode('decomposeMatrix', name='mouth_M_%sMtx2Srt_utl' % name)
    ctrl.worldMatrix[0].connect(mtx2Srt.inputMatrix)

    return{'ctrl':ctrl, 'buffer':buffer, 'mp':mp, 'mtx2Srt':mtx2Srt, 'pb':pb}

def makeTangent(parentCtrl, name, offsetY=1, invert=0):
    ctrl = drControls.ballCtrl(radius = ctrlSize*.5)
    ctrl.rename('mouth_M_%s_tangent_ctrl' % name)
    buffer = coreUtils.addParent(ctrl, 'group', name=ctrl.name().replace('ctrl', 'ctrl_zeroSrt'))
    buffer.setParent(parentCtrl['ctrl'])
    buffer.t.set((0,offsetY, 0))
    buffer.s.set((1, 1, 1))
    buffer.r.set((0, 0, 0))
    vp = coreUtils.pointMatrixMult(ctrl.t, parentCtrl['ctrl'].worldMatrix[0], name='mouth_M_%sTangentWorldPos_utl' % name)
    return {'ctrl':ctrl, 'buffer':buffer, 'vp':vp}

rCorner = makeCtrl(name='R_corner', invert=1, minParam=0, maxParam=1, startParam=0.325)
lCorner = makeCtrl(name='L_corner', invert=0, minParam=0, maxParam=1, startParam=0.675)

tMid = makeCtrl(name='T_mid', invert=0, minParam=rCorner['mp'].uValue, maxParam=lCorner['mp'].uValue, startParam=0.5)
bMid = makeCtrl(name='B_mid', invert=0, minParam=rCorner['mp'].uValue, maxParam=lCorner['mp'].uValue, startParam=0.5)

trOuter = makeCtrl(name='T_R_outer', invert=1, minParam=rCorner['mp'].uValue, maxParam=tMid['mp'].uValue, startParam=0.33)
trInner = makeCtrl(name='T_R_inner', invert=1, minParam=rCorner['mp'].uValue, maxParam=tMid['mp'].uValue, startParam=0.67)

tlOuter = makeCtrl(name='T_L_outer', invert=0, minParam=lCorner['mp'].uValue, maxParam=tMid['mp'].uValue, startParam=0.33)
tlInner = makeCtrl(name='T_L_inner', invert=0, minParam=lCorner['mp'].uValue, maxParam=tMid['mp'].uValue, startParam=0.67)

brOuter = makeCtrl(name='B_R_outer', invert=1, minParam=rCorner['mp'].uValue, maxParam=bMid['mp'].uValue, startParam=0.33)
brInner = makeCtrl(name='B_R_inner', invert=1, minParam=rCorner['mp'].uValue, maxParam=bMid['mp'].uValue, startParam=0.67)

blOuter = makeCtrl(name='B_L_outer', invert=0, minParam=lCorner['mp'].uValue, maxParam=bMid['mp'].uValue, startParam=0.33)
blInner = makeCtrl(name='B_L_inner', invert=0, minParam=lCorner['mp'].uValue, maxParam=bMid['mp'].uValue, startParam=0.67)

trTangent = makeTangent(parentCtrl=rCorner, name='T_R', offsetY=ctrlSize*1.5, invert=1)
brTangent = makeTangent(parentCtrl=rCorner, name='B_R', offsetY=ctrlSize*-1.5, invert=1)

tlTangent = makeTangent(parentCtrl=lCorner, name='T_L', offsetY=ctrlSize*1.5)
blTangent = makeTangent(parentCtrl=lCorner, name='B_L', offsetY=ctrlSize*-1.5)

# Set up jaw follow
def follow(min, max, mult, slave, name):
    m = pmc.createNode('blendColors', name='mouth_M_%s_jawFollowBlend_utl' % name)
    if type(min) == pmc.general.Attribute:
        min.connect(m.color1R)
    else:
        m.color1R.set(min)
    if type(max) == pmc.general.Attribute:
        max.connect(m.color2R)
    else:
        m.color2R.set(max)
    m.blender.set(mult)
    m.outputR.connect(slave)
    return m

# Attrs for follow and zip
pmc.addAttr(rCorner['ctrl'], ln='followJaw', at='float', k=1, h=0, minValue=0, maxValue=1)
rCorner['ctrl'].followJaw.connect(rCorner['pb'].weight)
pmc.addAttr(rCorner['ctrl'], ln='zip', at='float', k=1, h=0, minValue=0, maxValue=1)
pmc.addAttr(rCorner['ctrl'], ln='zipWidth', at='float', k=1, h=0, minValue=0.1, maxValue=0.5)
#pmc.addAttr(rCorner['ctrl'], ln='zipPosition', at='float', k=1, h=0, minValue=-1.0, maxValue=1.0)
pmc.addAttr(rCorner['ctrl'], ln='zipHeight', at='float', k=1, h=0, minValue=0, maxValue=1)
pmc.addAttr(rCorner['ctrl'], ln='squetch', at='float', k=1, h=0, minValue=0, maxValue=1)

pmc.addAttr(lCorner['ctrl'], ln='followJaw', at='float', k=1, h=0, minValue=0, maxValue=1)
lCorner['ctrl'].followJaw.connect(lCorner['pb'].weight)
pmc.addAttr(lCorner['ctrl'], ln='zip', at='float', k=1, h=0, proxy=rCorner['ctrl'].zip)
pmc.addAttr(lCorner['ctrl'], ln='zipWidth', at='float', k=1, h=0, proxy=rCorner['ctrl'].zipWidth)
#pmc.addAttr(lCorner['ctrl'], ln='zipPosition', at='float', k=1, h=0, proxy=rCorner['ctrl'].zipPosition)
pmc.addAttr(lCorner['ctrl'], ln='zipHeight', at='float', k=1, h=0, proxy=rCorner['ctrl'].zipHeight)
pmc.addAttr(lCorner['ctrl'], ln='squetch', at='float', k=1, h=0, proxy=rCorner['ctrl'].squetch)



follow(0.0, rCorner['ctrl'].followJaw, .5, trOuter['pb'].weight, 'T_R_outer')
follow(1.0, rCorner['ctrl'].followJaw, .5, brOuter['pb'].weight, 'B_R_outer')
follow(0.0, rCorner['ctrl'].followJaw, .9, trInner['pb'].weight, 'T_R_inner')
follow(1.0, rCorner['ctrl'].followJaw, .9, brInner['pb'].weight, 'B_R_inner')

follow(0.0, lCorner['ctrl'].followJaw, .5, tlOuter['pb'].weight, 'T_L_outer')
follow(1.0, lCorner['ctrl'].followJaw, .5, blOuter['pb'].weight, 'B_L_outer')
follow(0.0, lCorner['ctrl'].followJaw, .9, tlInner['pb'].weight, 'T_L_inner')
follow(1.0, lCorner['ctrl'].followJaw, .9, blInner['pb'].weight, 'B_L_inner')

tMid['pb'].weight.set(0.0)
bMid['pb'].weight.set(1.0)


# Make lip curves
points = [(i, 0, 0) for i in range(9)]

topCtrls = [rCorner, trTangent, trOuter, trInner, tMid, tlInner, tlOuter, tlTangent, lCorner]
topCrv = curveUtils.curveThroughPoints(positions=points, name='mouth_M_top_lip', degree=2, bezier=0, rebuild=1)
for i in range(9):
    try:
        topCtrls[i]['mtx2Srt'].outputTranslate.connect(topCrv.controlPoints[i])
    except:
        topCtrls[i]['vp'].output.connect(topCrv.controlPoints[i])

btmCtrls = [rCorner, brTangent, brOuter, brInner, bMid, blInner, blOuter, blTangent, lCorner]
btmCrv = curveUtils.curveThroughPoints(positions=points, name='mouth_M_btm_lip', degree=2, bezier=0, rebuild=1)
for i in range(9):
    try:
        btmCtrls[i]['mtx2Srt'].outputTranslate.connect(btmCrv.controlPoints[i])
    except:
        btmCtrls[i]['vp'].output.connect(btmCrv.controlPoints[i])

# Stretch
topCrv_inf = pmc.createNode('curveInfo', name='mouth_M_topCrvInfo_utl')
topCrv.worldSpace[0].connect(topCrv_inf.inputCurve)
topCrv_restLength = coreUtils.multiplyDouble(topCrv_inf.arcLength.get(), scaleAttr, name='mouth_M_topCrvRestLength_utl')
topCrvStretch =coreUtils.divide(topCrv_restLength.output, topCrv_inf.arcLength, name='mouth_M_topCrvStretch_utl')
topStretchedScale = coreUtils.multiplyDouble(scaleAttr, topCrvStretch.outputX, name='mouth_M_topStretchedScale_utl')

btmCrv_inf = pmc.createNode('curveInfo', name='mouth_M_btmCrvInfo_utl')
btmCrv.worldSpace[0].connect(btmCrv_inf.inputCurve)
btmCrv_restLength = coreUtils.multiplyDouble(btmCrv_inf.arcLength.get(), scaleAttr, name='mouth_M_btmCrvRestLength_utl')
btmCrvStretch =coreUtils.divide(btmCrv_restLength.output, btmCrv_inf.arcLength, name='mouth_M_btmCrvStretch_utl')
btmStretchedScale = coreUtils.multiplyDouble(scaleAttr, btmCrvStretch.outputX, name='mouth_M_btmStretchedScale_utl')

# Make joints

# Corners & tangents
rCornerJ = pmc.createNode('joint', name = 'mouth_M_R_corner_dfm')
rCorner['mtx2Srt'].outputTranslate.connect(rCornerJ.t)
rCorner['mtx2Srt'].outputRotate.connect(rCornerJ.r)

trTangentJ = pmc.createNode('joint', name='mouth_M_R_T_tangent_dfm')
trTangent['vp'].output.connect(trTangentJ.t)
rCorner['mtx2Srt'].outputRotate.connect(trTangentJ.r)

pmc.select(rCornerJ)
brTangentJ = pmc.createNode('joint', name='mouth_M_R_B_tangent_dfm')
brTangent['vp'].output.connect(brTangentJ.t)
rCorner['mtx2Srt'].outputRotate.connect(brTangentJ.r)

pmc.select()
lCornerJ = pmc.createNode('joint', name = 'mouth_M_L_corner_dfm')
lCorner['mtx2Srt'].outputTranslate.connect(lCornerJ.t)
lCorner['mtx2Srt'].outputRotate.connect(lCornerJ.r)

tlTangentJ = pmc.createNode('joint', name='mouth_M_L_T_tangent_dfm')
tlTangent['vp'].output.connect(tlTangentJ.t)
lCorner['mtx2Srt'].outputRotate.connect(tlTangentJ.r)

pmc.select(lCornerJ)
blTangentJ = pmc.createNode('joint', name='mouth_M_L_B_tangent_dfm')
blTangent['vp'].output.connect(blTangentJ.t)
lCorner['mtx2Srt'].outputRotate.connect(blTangentJ.r)

# Path joints
# upVectors
topRow2 = coreUtils.matrixAxisToVector(crv, 'mouth_M_topRailMtx_row2_utl', axis='y')
btmRow2 = coreUtils.matrixAxisToVector(jawCrv, 'mouth_M_btmRailMtx_row2_utl', axis='y')

# Roll
pmc.addAttr(rCorner['ctrl'], ln='roll_top', at='doubleAngle', k=1, h=0)
pmc.addAttr(rCorner['ctrl'], ln='roll_btm', at='doubleAngle', k=1, h=0)
pmc.addAttr(lCorner['ctrl'], ln='roll_top', at='doubleAngle', k=1, h=0)
pmc.addAttr(lCorner['ctrl'], ln='roll_btm', at='doubleAngle', k=1, h=0)

def makeMp(path, upVec, param, name, rollAttr):
    mp = pmc.createNode('motionPath', name='mouth_M_%s_mp_utl' % name)
    path.worldSpace[0].connect(mp.geometryPath)
    mp.fractionMode.set(1)
    mp.frontAxis.set(0)
    mp.upAxis.set(1)
    upVec.connect(mp.worldUpVector)
    mp.uValue.set(param)

    # Rolling
    rollSum = pmc.createNode('animBlendNodeAdditiveDA', name='mouth_M_%s_rollSum_utl' % name)
    rRoll = pmc.Attribute('%s.%s' % (rCorner['ctrl'].name(), rollAttr))
    lRoll = pmc.Attribute('%s.%s' % (lCorner['ctrl'].name(), rollAttr))
    rRoll.connect(rollSum.inputA)
    lRoll.connect(rollSum.inputB)
    if rollAttr == 'roll_btm':
        rollSum.weightA.set(-1.0 + param)
        rollSum.weightB.set(-param)
    else:
        rollSum.weightA.set(1.0 - param)
        rollSum.weightB.set(param)

    normal = coreUtils.matrixAxisToVector(mp.orientMatrix, 'mouth_M_%s_normalVec_utl' % name, axis='z')
    tangent = coreUtils.cross(upVec, normal.output, 'mouth_M_%s_tangentVec_utl' % name)
    m = coreUtils.matrixFromVectors(tangent.output, upVec, normal.output, upVec, 'mouth_M_%s_rotateMtx_utl' % name)
    mtx2Srt = pmc.createNode('decomposeMatrix', name='mouth_M_%s_rotateMtx2Srt_utl' % name)
    m.output.connect(mtx2Srt.inputMatrix)

    # Squash n stretch


    return {'mp':mp, 'mtx2Srt':mtx2Srt, 'roll':rollSum}


topMps=[]
for i in range(numJoints):
    if i == 0 or i == numJoints-1:
        continue
    num = str(i).zfill(2)
    param = (1.0 / (numJoints-1))*i
    mp = makeMp(topCrv,topRow2.output, param, '%s_topLip' % num, 'roll_top')
    topMps.append(mp)

btmMps = []
for i in range(numJoints):
    if i == 0 or i == numJoints-1:
        continue
    num = str(i).zfill(2)
    param = (1.0 / (numJoints-1))*i
    mp = makeMp(btmCrv,btmRow2.output, param, '%s_btmLip' % num,  'roll_btm')
    btmMps.append(mp)

# Blended srts
topBlends = []
btmBlends = []

for i in range(numJoints):
    if i == 0 or i == numJoints-1:
        continue
    param = (1.0 / (numJoints-1))*i
    num = str(i).zfill(2)
    topMps[i-1]['mp'].uValue.get()
    rv = pmc.createNode('remapValue', name='mouth_M_%s_zipFunction_utl' % num)
    rCorner['ctrl'].zip.connect(rv.inputValue)
    zipEnd = coreUtils.add([0.5 -(abs(0.5 - param)), rCorner['ctrl'].zipWidth], name='mouth_M_T_%s_zipEnd_utl' % num)
    rv.value[0].value_Interp.set(2)
    rv.value[0].value_Position.set(0.5 -(abs(0.5 - param)))
    rv.value[1].value_Interp.set(2)
    zipEnd.output1D.connect(rv.value[1].value_Position)
    pb = coreUtils.pairBlend(inTranslate1=topMps[i-1]['mp'].allCoordinates,
                             inRotate1=topMps[i-1]['mtx2Srt'].outputRotate,
                             inTranslate2=btmMps[i-1]['mp'].allCoordinates,
                             inRotate2=btmMps[i-1]['mtx2Srt'].outputRotate,
                             name='mouth_M_%s_zipHeightBlend_utl' % num,
                             blendAttr=rCorner['ctrl'].zipHeight)

    topPb = coreUtils.pairBlend(inTranslate1=topMps[i-1]['mp'].allCoordinates,
                             inRotate1=topMps[i-1]['mtx2Srt'].outputRotate,
                             inTranslate2=pb.outTranslate,
                             inRotate2=pb.outRotate,
                             name='mouth_M_T_%s_zipBlend_utl' % num,
                             blendAttr=rv.outValue)

    btmPb = coreUtils.pairBlend(inTranslate1=btmMps[i-1]['mp'].allCoordinates,
                             inRotate1=btmMps[i-1]['mtx2Srt'].outputRotate,
                             inTranslate2=pb.outTranslate,
                             inRotate2=pb.outRotate,
                             name='mouth_M_T_%s_zipBlend_utl' % num,
                             blendAttr=rv.outValue)

    pb.rotInterpolation.set(1)

    topPb.rotInterpolation.set(1)
    topBlends.append(topPb)

    btmPb.rotInterpolation.set(1)
    btmBlends.append(btmPb)

# tweakers
for i in range(numJoints):
    if i == 0 or i == numJoints-1:
        continue
    param = (1.0 / (numJoints-1))*i
    num = str(i).zfill(2)

    squetchMult = coreUtils.multiplyDouble((0.5-(abs(0.5-param)))*2, rCorner['ctrl'].squetch, name='mouth_M_T_%s_squetchMult_utl' % num)
    topSquetch = coreUtils.blend(topStretchedScale.output, scaleAttr, name='mouth_M_T_%s_squetchBlend_utl' % num, blendAttr=squetchMult.output)

    # ctrl
    topCtrl = drControls.ballCtrl(radius = ctrlSize*.25, name='mouth_M_T_%s_ctrl' % num)

    # buffer
    topBuffer = coreUtils.addParent(topCtrl, 'group', name='mouth_M_T_%s_ctrl_zeroSrt' % num)
    topBlends[i-1].outTranslate.connect(topBuffer.t)
    topBlends[i-1].outRotateY.connect(topBuffer.ry)
    topBlends[i-1].outRotateZ.connect(topBuffer.rz)
    topRolledX = pmc.createNode('animBlendNodeAdditiveDA', name='mouth_M_T_%s_rolled_utl' % num)
    topBlends[i-1].outRotateX.connect(topRolledX.inputA)
    topMps[i-1]['roll'].output.connect(topRolledX.inputB)
    topRolledX.output.connect(topBuffer.rx)

    btmSquetch = coreUtils.blend(btmStretchedScale.output, scaleAttr, name='mouth_M_B_%s_squetchBlend_utl' % num, blendAttr=squetchMult.output)

    # ctrl
    btmCtrl = drControls.ballCtrl(radius = ctrlSize*.25, name='mouth_M_B_%s_ctrl' % num)

    btmBuffer = coreUtils.addParent(btmCtrl, 'group', name='mouth_M_B_%s_ctrl_zeroSrt' % num)
    btmBlends[i-1].outTranslate.connect(btmBuffer.t)
    btmBlends[i-1].outRotateY.connect(btmBuffer.ry)
    btmBlends[i-1].outRotateZ.connect(btmBuffer.rz)
    btmRolledX = pmc.createNode('animBlendNodeAdditiveDA', name='mouth_M_B_%s_rolled_utl' % num)
    btmBlends[i-1].outRotateX.connect(btmRolledX.inputA)
    btmMps[i-1]['roll'].output.connect(btmRolledX.inputB)
    btmRolledX.output.connect(btmBuffer.rx)

    # collisions
    coreUtils.defineCollision(topCtrl, btmCtrl, settingsNode=topCtrl, name='mouth_M_%s_collision' % num, axis='y')

    # Joints
    topJ = pmc.createNode('joint', name='mouth_M_T_%s_dfm' % num)
    d = coreUtils.isDecomposed(topCtrl)
    vp = coreUtils.pointMatrixMult((0,0,0), topCtrl.worldMatrix[0], name='mouth_M_T_%s_collisionOffset_utl' % num)
    topCtrl.end_push.connect(vp.input1Y)
    vp.output.connect(topJ.t)
    d.outputRotate.connect(topJ.r)
    topSquetch.outputR.connect(topJ.sy)
    scaleAttr.connect(topJ.sx)
    collisionScale = coreUtils.add([topSquetch.outputR, topCtrl.secondary_push], name='mouth_M_T_%s_collisionScale_utl' % num)
    collisionScale.output1D.connect(topJ.sz)

    btmJ = pmc.createNode('joint', name='mouth_M_B_%s_dfm' % num)
    d = coreUtils.isDecomposed(btmCtrl)
    vp = coreUtils.pointMatrixMult((0,0,0), btmCtrl.worldMatrix[0], name='mouth_M_B_%s_collisionOffset_utl' % num)
    topCtrl.start_push.connect(vp.input1Y)
    vp.output.connect(btmJ.t)
    d.outputRotate.connect(btmJ.r)
    btmSquetch.outputR.connect(btmJ.sy)
    scaleAttr.connect(btmJ.sx)
    collisionScale = coreUtils.add([btmSquetch.outputR, topCtrl.secondary_push], name='mouth_M_B_%s_collisionScale_utl' % num)
    collisionScale.output1D.connect(btmJ.sz)














