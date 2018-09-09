import pymel.core as pmc
import drRigging.utils.coreUtils as coreUtils
import drRigging.utils.curveUtils as curveUtils
import drRigging.objects.controls as controls
import drRigging.nexus.artStickers_faceRigging as faceRigging
reload(faceRigging)

reload(curveUtils)
reload(coreUtils)
reload(controls)

configDict = {
    'radius':5,
    'spread':30,
    'height':.25,
}

def addOutputAttrs(node):
    try:
        pmc.deleteAttr(node.outMatrix)
    except:
        print 'attributes to delete not found'

    pmc.select(node)
    pmc.addAttr(ln='outMatrix', at='matrix', multi=1)
    #pmc.addAttr(ln='outMatrix', at='matrix', parent='outMatrix')

def buildBaseRig(name='eye', numTweaks=7, tweaks=1):
    '''
    Create the bezier setup with corner, top and bottom controls.
    Radius sets the distance from the spin pivots to the rig
    Spread sets the width of the corners (in degrees from the centre)
    '''
    # Base structure
    rootGrp = pmc.group(empty=1, name='%s_cmpnt' % name)
    pmc.addAttr(rootGrp, ln='rigParent', dt='string')
    rootGrp.rigParent.set('components_GRP')

    inputGrp = coreUtils.addChild(rootGrp, 'group', name='%s_input' % name)
    pmc.select(inputGrp)
    pmc.addAttr(ln='rigInputs', dt='string', multi=1)
    pmc.addAttr(inputGrp, at='matrix', ln='head_in_mtx')
    pmc.addAttr(inputGrp, at='matrix', ln='jaw_in_mtx')

    controlsGrp = coreUtils.addChild(rootGrp, 'group', name='%s_controls' % name)
    pmc.select(controlsGrp)
    pmc.addAttr(ln='rigInputs', dt='string', multi=1)
    controlsGrp.rigInputs[0].set('options_CTL.all_ctls_vis>>visibility')

    rigGrp = coreUtils.addChild(rootGrp, 'group', name='%s_rig' % name)
    rigGrp.visibility.set(0)

    outputGrp = coreUtils.addChild(rootGrp, 'group', name='%s_output' % name)
    addOutputAttrs(outputGrp)

    pmc.addAttr(outputGrp, ln='outputType', dt='string')
    # in case there is a discrepancy between the neck joint and the head control
    pmc.addAttr(outputGrp, ln='ctrl2Joint_offset_mtx', at='matrix')
    outputGrp.outputType.set('eye')

    # Config node
    config = coreUtils.addChild(controlsGrp, 'group', name='%s_config' % name)
    pmc.addAttr(config, ln='radius', at='double', k=0, h=0)
    config.radius.set(configDict['radius'])

    pmc.addAttr(config, ln='spread', at='doubleAngle', k=0, h=0)
    config.spread.set(configDict['spread'])

    pmc.addAttr(config, ln='height', at='double', k=0, h=0)
    config.height.set(configDict['height'])

    # Base srts
    rootSrt = coreUtils.addChild(controlsGrp, 'group', name='%s_root_srt' % name)
    headSrt = coreUtils.decomposeMatrix(inputGrp.head_in_mtx, name='%s_head_in_mtx2Srt_utl' % name)
    coreUtils.connectDecomposedMatrix(headSrt, rootSrt)

    centreSrt = coreUtils.addChild(rootSrt, 'group', name='%s_centre_srt' % name)
    negScaleSrt = coreUtils.addChild(centreSrt, 'group', name='%s_negScale_srt' % name)
    negScaleSrt.sx.set(-1)

    # Set up jaw blending stuff
    jawOffset = pmc.createNode('composeMatrix', name='%s_jawOffsetMtx_utl' % name)
    jawResultMtx = pmc.createNode('multMatrix', name='%s_jawResultMtx_utl' % name)
    jawOffset.outputMatrix.connect(jawResultMtx.matrixIn[0])
    inputGrp.jaw_in_mtx.connect(jawResultMtx.matrixIn[1])
    centreSrt.worldInverseMatrix[0].connect(jawResultMtx.matrixIn[2])
    jawSrt = coreUtils.decomposeMatrix(jawResultMtx.matrixSum, name='%s_jawResultMtx2Srt_utl' % name)
    jawRotConvert = coreUtils.convert(jawSrt.outputRotateX, 1.0, name='%s_jawRotAngle2Float_utl' % name)

    # Spin srt + Ctrl structure
    ctrls = []
    constGrps = []
    for i in ['R', 'T', 'L', 'B']:
        aimDict = {'T':'up', 'L':'left', 'B':'down', 'R':'left'}
        constSrt = coreUtils.addChild(centreSrt, 'group', name='%s_%s_const_srt' % (name, i))
        constGrps.append(constSrt)
        spinSrt = coreUtils.addChild(constSrt, 'group', name='%s_%s_spin_srt' % (name, i))

        bufferSrt = coreUtils.addChild(spinSrt, 'group', name='%s_%s_ctrlBufferSrt' % (name, i))
        negSrt = coreUtils.addChild(bufferSrt, 'group', name='%s_%s_xNegSrt' % (name, i))
        ctrl = controls.triCtrl(size=config.radius.get()*.2, aim=aimDict[i], name='%s_%s_ctrl' % (name, i))
        pmc.addAttr(ctrl, ln='follow_jaw', minValue=0.0, maxValue=1.0, k=1, h=0, at='double')
        ctrl.setParent(negSrt)
        config.radius.connect(bufferSrt.tz)
        if i == 'T':
            config.height.connect(bufferSrt.ty)
        elif i == 'B':
            heightInvert = coreUtils.convert(config.height, -1, name='%s_heightInvert_utl' % name)
            heightInvert.output.connect(bufferSrt.ty)

        # Negate x & y translate and connect to spin
        conv = coreUtils.convert(ctrl.tx, -1, name='%s_%s_invertTX_utl' % (name, i))
        conv.output.connect(negSrt.tx)
        spinFac = coreUtils.convert(config.radius, 6.283, name='%s_%s_spinFactor_utl' % (name, i))
        spinAmount = coreUtils.divide(ctrl.tx, spinFac.output, name='%s_%s_spinAmountX_utl' % (name, i))
        spinConv = coreUtils.convert(spinAmount.outputX, 6.283, name='%s_%s_spinX_utl' % (name, i))

        conv = coreUtils.convert(ctrl.ty, -1, name='%s_%s_invertTY_utl' % (name, i))
        conv.output.connect(negSrt.ty)
        spinAmountY = coreUtils.divide(ctrl.ty, spinFac.output, name='%s_%s_spinAmountY_utl' % (name, i))
        spinConvY = coreUtils.convert(spinAmountY.outputX, -6.283, name='%s_%s_spinY_utl' % (name, i))


        if i == 'R':
            negScaleSrt.setParent(constSrt)
            spinSrt.setParent(negScaleSrt)
            spinSrt.s.set(1,1,1)
            spinSrt.r.set(0,0,0)
        if i == 'L' or i == 'R':
            spinOffset = pmc.createNode('animBlendNodeAdditiveDA', name='%s_%s_spinOffset_utl' % (name, i))
            spinConv.output.connect(spinOffset.inputA)
            config.spread.connect(spinOffset.inputB)
            spinOffset.output.connect(spinSrt.ry)
        else:
            spinConv.output.connect(spinSrt.ry)
        spinConvY.output.connect(spinSrt.rx)
        spinSrt.ro.set(2)

        # set up jaw blending
        jawBlend = coreUtils.pairBlend((0,0,0), (0,0,0), jawSrt.outputTranslate, jawSrt.outputRotate, name='%s_%s_jawBlend_utl' % (name, i), blendAttr=ctrl.follow_jaw)
        jawBlend.outTranslate.connect(constSrt.t)
        jawBlend.outRotate.connect(constSrt.r)

        # set up corner push on jaw open
        if i == 'R' or i =='L':
            pmc.addAttr(bufferSrt, ln='jawOpenPush', at='double', k=0, h=0)
            rv = pmc.createNode('remapValue', name='%s_%s_jawPushRemap_utl' % (name, i))
            jawAdd = pmc.createNode('animBlendNodeAdditive', name='%s_%s_jawOpenAdd_utl' % (name, i))
            config.radius.connect(jawAdd.inputB)
            rv.outValue.connect(jawAdd.weightA)
            ctrl.follow_jaw.connect(rv.inputValue)
            jawPushMult = coreUtils.multiplyDouble(jawRotConvert.output, bufferSrt.jawOpenPush, name='%s_%s_jawPushMult' % (name, i))
            jawPushMult.output.connect(jawAdd.inputA)
            jawAdd.output.connect(bufferSrt.tz, f=1)
            rv.value[1].value_Position.set(.5)
            rv.value[2].value_FloatValue.set(0)
            rv.value[2].value_Position.set(1)
            rv.value[0].value_Interp.set(2)
            rv.value[1].value_Interp.set(2)


        ctrls.append(ctrl)

    # Distance nodes for auto scaling of tangents
    dist_R_T = coreUtils.distanceBetweenNodes(ctrls[0], ctrls[1], name='%s_dist_R_T_utl' % name)
    scaledDist_R_T = coreUtils.divide(dist_R_T.distance, headSrt.outputScaleX, name='%s_scaledDist_R_T_utl' % name)

    dist_L_T = coreUtils.distanceBetweenNodes(ctrls[2], ctrls[1], name='%s_dist_L_T_utl' % name)
    scaledDist_L_T = coreUtils.divide(dist_L_T.distance, headSrt.outputScaleX, name='%s_scaledDist_L_T_utl' % name)

    dist_R_B = coreUtils.distanceBetweenNodes(ctrls[0], ctrls[3], name='%s_dist_R_B_utl' % name)
    scaledDist_R_B = coreUtils.divide(dist_R_B.distance, headSrt.outputScaleX, name='%s_scaledDist_R_B_utl' % name)

    dist_L_B = coreUtils.distanceBetweenNodes(ctrls[2], ctrls[3], name='%s_dist_L_B_utl' % name)
    scaledDist_L_B = coreUtils.divide(dist_L_B.distance, headSrt.outputScaleX, name='%s_scaledDist_L_B_utl' % name)

    scaledHeight = coreUtils.divide(config.height, headSrt.outputScaleY, name='%s_scaledHeight_utl' % name)


    # Build curves
    points = [(0,0,0) for i in range(7)]
    knots = [0,0,0,1,1,1,2,2,2]
    topCrv = pmc.curve(d=3, p=points, k=knots, name='%s_T_macro_crv' % name)
    topCrv.setParent(rigGrp)
    btmCrv = pmc.curve(d=3, p=points, k=knots, name='%s_B_macro_crv' % name)
    btmCrv.setParent(rigGrp)

    p1 = coreUtils.pointMatrixMult((0,0,0), ctrls[0].worldMatrix[0], name='%s_R_pos_utl' % name)

    p2TanLen = coreUtils.multiplyDouble(headSrt.outputScaleY, scaledHeight.outputX, name='%s_R_outTan_len_utl' % name)
    p2 = coreUtils.pointMatrixMult((0,0,0), ctrls[0].worldMatrix[0], name='%s_R_outTan_pos_utl' % name)
    p2TanLen.output.connect(p2.input1Y)

    p3TanLen = coreUtils.multiply(scaledDist_R_T.outputX, -.5, name='%s_T_inTan_len_utl' % name)
    p3 = coreUtils.pointMatrixMult(p3TanLen.output, ctrls[1].worldMatrix[0], name='%s_T_inTan_pos_utl' % name)

    p4 = coreUtils.pointMatrixMult((0,0,0), ctrls[1].worldMatrix[0], name='%s_T_pos_utl' % name)

    p5TanLen = coreUtils.multiply(scaledDist_L_T.outputX, .5, name='%s_T_outTan_len_utl' % name)
    p5 = coreUtils.pointMatrixMult(p5TanLen.output, ctrls[1].worldMatrix[0], name='%s_T_outTan_pos_utl' % name)

    p6 = coreUtils.pointMatrixMult((0,0,0), ctrls[2].worldMatrix[0], name='%s_L_inTan_pos_utl' % name)
    p2TanLen.output.connect(p6.input1Y)

    p7 = coreUtils.pointMatrixMult((0,0,0), ctrls[2].worldMatrix[0], name='%s_L_pos_utl' % name)

    p8TanLen = coreUtils.convert(p2TanLen.output, -1, name='%s_L_outTan_len_utl' % name)
    p8 = coreUtils.pointMatrixMult((0,0,0), ctrls[2].worldMatrix[0], name='%s_L_outTan_pos_utl' % name)
    p8TanLen.output.connect(p8.input1Y)

    p9TanLen = coreUtils.multiply(scaledDist_L_B.outputX, .5, name='%s_B_inTan_len_utl' % name)
    p9 = coreUtils.pointMatrixMult(p9TanLen.output, ctrls[3].worldMatrix[0], name='%s_B_inTan_pos_utl' % name)

    p10 = coreUtils.pointMatrixMult((0,0,0), ctrls[3].worldMatrix[0], name='%s_B_pos_utl' % name)

    p11TanLen = coreUtils.multiply(scaledDist_R_B.outputX, -.5, name='%s_B_outTan_len_utl' % name)
    p11 = coreUtils.pointMatrixMult(p11TanLen.output, ctrls[3].worldMatrix[0], name='%s_B_outTan_pos_utl' % name)

    p12 = coreUtils.pointMatrixMult((0,0,0), ctrls[0].worldMatrix[0], name='%s_R_inTan_pos_utl' % name)
    p8TanLen.output.connect(p12.input1Y)

    p1.output.connect(topCrv.controlPoints[0])
    p2.output.connect(topCrv.controlPoints[1])
    p3.output.connect(topCrv.controlPoints[2])
    p4.output.connect(topCrv.controlPoints[3])
    p5.output.connect(topCrv.controlPoints[4])
    p6.output.connect(topCrv.controlPoints[5])
    p7.output.connect(topCrv.controlPoints[6])

    p1.output.connect(btmCrv.controlPoints[0])
    p12.output.connect(btmCrv.controlPoints[1])
    p11.output.connect(btmCrv.controlPoints[2])
    p10.output.connect(btmCrv.controlPoints[3])
    p9.output.connect(btmCrv.controlPoints[4])
    p8.output.connect(btmCrv.controlPoints[5])
    p7.output.connect(btmCrv.controlPoints[6])

    returnDict = {
        'root':rootSrt,
        'topCrv':topCrv,
        'btmCrv':btmCrv,
        'tangents':[p2, p8],
        'rigGrp':rigGrp
    }

    if not tweaks:
        return returnDict

    ##------------------------------TWEAKS-----------------------------------------##

    # Create motions path nodes
    tweakCtrls = []
    tweakGrp = coreUtils.addChild(centreSrt, 'group', name='%s_tweakControls_hrc' % name)
    for i in range(numTweaks):
        num = str(i+1).zfill(2)
        mp = pmc.createNode('motionPath', name='%s_T_%s_tweakMotionPath_utl' % (name, num))
        topCrv.worldSpace[0].connect(mp.geometryPath)
        mp.uValue.set(1.0 / (numTweaks-1) * i)
        bfr = pmc.group(empty=1, name='%s_tweak_%s_bufferSrt' % (name, num))
        aim = 'up'
        if i==numTweaks-1 or i == 0:
            aim='left'
        ctrl = controls.triCtrl(size = configDict['radius']*.075, aim=aim, name='%s_tweak_%s_ctrl' % (name, num))
        ctrl.setParent(bfr)
        bfrLocalPos = coreUtils.pointMatrixMult(mp.allCoordinates, centreSrt.worldInverseMatrix[0], name='%s_tweakLocalPos_%s_utl' % (name, num))
        bfrLocalPos.output.connect(bfr.t)
        bfrAngle = pmc.createNode('angleBetween', name='%s_tweakAngle_%s_utl' % (name, num))
        bfrLocalPos.output.connect(bfrAngle.vector2)
        bfrAngle.vector1.set((0,0,1))
        euler2Quat = pmc.createNode('eulerToQuat', name='%s_tweakAngleEuler2Quat_%s_utl' % (name, num))
        bfrAngle.euler.connect(euler2Quat.inputRotate)
        quatToEuler = pmc.createNode('quatToEuler', name='%s_tweakAngleQuat2Euler_%s_utl' % (name, num))
        euler2Quat.outputQuat.connect(quatToEuler.inputQuat)
        quatToEuler.inputRotateOrder.set(2)
        bfr.ro.set(2)
        quatToEuler.outputRotateX.connect(bfr.rx)
        quatToEuler.outputRotateY.connect(bfr.ry)
        mp.fractionMode.set(1)
        tweakCtrls.append(ctrl)
        d = coreUtils.decomposeMatrix(ctrl.worldMatrix[0], name='%s_tweak_%s_mtx2Srt_utl' % (name, num))
        if i==(numTweaks-1):
            tanMult = coreUtils.convert(p8.input1Y, -0.5, name='%s_T_tweakInTan_utl' % name)
            b = coreUtils.addChild(tweakCtrls[-1], 'group', name=bfr.name().replace('bufferSrt', 'inTan_bufferSrt'))
            tanMult.output.connect(b.ty)
            c = controls.triCtrl(size = configDict['radius']*.05, aim='up', name='%s_tweak_%s_inTan_ctrl' % (name, num))
            coreUtils.align(c, b, parent=1)
            tweakCtrls.append(c)
        if i==0:
            tanMult = coreUtils.convert(p2.input1Y, 0.5, name='%s_T_tweakOutTan_utl' % name)
            b = coreUtils.addChild(tweakCtrls[0], 'group', name=bfr.name().replace('bufferSrt', 'outTan_bufferSrt'))
            tanMult.output.connect(b.ty)
            c = controls.triCtrl(size = configDict['radius']*.05, aim='up', name='%s_tweak_%s_outTan_ctrl' % (name, num))
            coreUtils.align(c, b, parent=1)
            tweakCtrls.append(c)

        if d.outputTranslateX.get() < 0.0:
            bfr.sx.set(-1)

        bfr.setParent(tweakGrp)

    for i in range(numTweaks)[1:-1]:
        num = str(i+numTweaks).zfill(2)
        mp = pmc.createNode('motionPath', name='%s_B_%s_tweakMotionPath_utl' % (name, num))
        btmCrv.worldSpace[0].connect(mp.geometryPath)
        mp.uValue.set(1.0 / (numTweaks-1) * i)
        bfr = pmc.group(empty=1, name='%s_tweak_%s_bufferSrt' % (name, num))
        ctrl = controls.triCtrl(size = configDict['radius']*.075, aim='down', name='%s_tweak_%s_ctrl' % (name, num))
        ctrl.setParent(bfr)
        bfrLocalPos = coreUtils.pointMatrixMult(mp.allCoordinates, centreSrt.worldInverseMatrix[0], name='%s_tweakLocalPos_%s_utl' % (name, num))
        bfrLocalPos.output.connect(bfr.t)
        bfrAngle = pmc.createNode('angleBetween', name='%s_tweakAngle_%s_utl' % (name, num))
        bfrLocalPos.output.connect(bfrAngle.vector2)
        bfrAngle.vector1.set((0,0,1))
        euler2Quat = pmc.createNode('eulerToQuat', name='%s_tweakAngleEuler2Quat_%s_utl' % (name, num))
        bfrAngle.euler.connect(euler2Quat.inputRotate)
        quatToEuler = pmc.createNode('quatToEuler', name='%s_tweakAngleQuat2Euler_%s_utl' % (name, num))
        euler2Quat.outputQuat.connect(quatToEuler.inputQuat)
        quatToEuler.inputRotateOrder.set(2)
        bfr.ro.set(2)
        quatToEuler.outputRotateX.connect(bfr.rx)
        quatToEuler.outputRotateY.connect(bfr.ry)
        mp.fractionMode.set(1)
        d = coreUtils.decomposeMatrix(ctrl.worldMatrix[0], name='%s_tweak_%s_mtx2Srt_utl' % (name, num))
        if i==1:
            tanMult = coreUtils.convert(p2.input1Y, -0.5, name='%s_B_tweakOutTan_utl' % name)
            b = coreUtils.addChild(tweakCtrls[0], 'group', name=tweakCtrls[0].name().replace('ctrl', 'inTan_bufferSrt'))
            tanMult.output.connect(b.ty)
            c = controls.triCtrl(size = configDict['radius']*.05, aim='down', name=tweakCtrls[0].name().replace('ctrl', 'inTan_ctrl'))
            coreUtils.align(c, b, parent=1)
            tweakCtrls.append(c)
        tweakCtrls.append(ctrl)
        if i==(numTweaks-2):
            tanMult = coreUtils.convert(p8.input1Y, 0.5, name='%s_B_tweakInTan_utl' % name)
            b = coreUtils.addChild(tweakCtrls[numTweaks], 'group', name=tweakCtrls[numTweaks].name().replace('ctrl', 'outTan_bufferSrt'))
            tanMult.output.connect(b.ty)
            c = controls.triCtrl(size = configDict['radius']*.05, aim='down', name=tweakCtrls[numTweaks].name().replace('ctrl', 'outTan_ctrl'))
            coreUtils.align(c, b, parent=1)
            tweakCtrls.append(c)

        if d.outputTranslateX.get() < 0.0:
            bfr.sx.set(-1)

        bfr.setParent(tweakGrp)

    ctrlSet = pmc.sets(ctrls + tweakCtrls, name='%s_ctls_SEL' % name)


    # CLEAN UP
    coreUtils.attrCtrl(nodeList=[ctrls[0], ctrls[2]], attrList=['sx', 'sz', 'visibility'])
    coreUtils.attrCtrl(nodeList=[ctrls[1], ctrls[3]], attrList=['sy', 'sz', 'visibility'])
    coreUtils.attrCtrl(nodeList=tweakCtrls, attrList=['sx', 'sy', 'sz', 'visibility'])
    coreUtils.attrCtrl(nodeList=ctrls + tweakCtrls, attrList=['aiRenderCurve', 'aiCurveWidth', 'aiSampleRate', 'aiCurveShaderR', 'aiCurveShaderG', 'aiCurveShaderB'])

    addOutputAttrs(outputGrp)
    joints = []
    # top joints
    for i in range(len(tweakCtrls)):
        num = str(i+1).zfill(2)
        ctrl = tweakCtrls[i]
        t = pmc.xform(ctrl, ws=1, q=1, t=1)[0]
        unMirror=0
        if t < 0.0:
            unMirror=1
        faceRigging.exposeOutput(ctrl, outputGrp, 'eye', unMirror=unMirror, createJoint=0)

        pmc.select(None)
        j = pmc.joint(name='%s_T_%s_Out_Jnt' % (name, num))
        joints.append(j)

        j.segmentScaleCompensate.set(0)
        pmc.addAttr(j, ln='jointIndex', at='short', k=0)
        j.jointIndex.set(i)
        pmc.addAttr(j, ln='rigType', dt='string')
        j.rigType.set('eye')

        d = coreUtils.decomposeMatrix(outputGrp.outMatrix[i], name='%s_outMtx2Srt_%s_utl' % (name, num))
        coreUtils.connectDecomposedMatrix(d, j)
