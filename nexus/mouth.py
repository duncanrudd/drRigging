import pymel.core as pmc
import drRigging.utils.coreUtils as coreUtils
import drRigging.utils.curveUtils as curveUtils
import drRigging.objects.controls as controls

reload(curveUtils)
reload(coreUtils)
reload(controls)

configDict = {
    'radius':2,
    'spread':30,
    'height':.25,
}

def buildBaseRig(name='mouth'):
    '''
    Create the bezier setup with corner, top and bottom controls.
    Radius sets the distance from the spin pivots to the rig
    Spread sets the width of the corners (in degrees from the centre)
    '''
    # Base structure
    rootGrp = pmc.group(empty=1, name='%s_cmpnt' % name)
    inputGrp = coreUtils.addChild(rootGrp, 'group', name='%s_input' % name)
    pmc.addAttr(inputGrp, at='matrix', ln='head_in_mtx')
    pmc.addAttr(inputGrp, at='matrix', ln='jaw_in_mtx')
    controlsGrp = coreUtils.addChild(rootGrp, 'group', name='%s_controls' % name)
    rigGrp = coreUtils.addChild(rootGrp, 'group', name='%s_rig' % name)
    outputGrp = coreUtils.addChild(rootGrp, 'group', name='%s_output' % name)

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
    centreSrt = coreUtils.addChild(rootSrt, 'group', name='%s_centre_srt' % name)
    negScaleSrt = coreUtils.addChild(centreSrt, 'group', name='%s_negScale_srt' % name)
    negScaleSrt.sx.set(-1)

    # Spin srt + Ctrl structure
    ctrls = []
    for i in ['R', 'T', 'L', 'B']:
        aimDict = {'T':'up', 'L':'left', 'B':'down', 'R':'left'}
        constSrt = coreUtils.addChild(centreSrt, 'group', name='%s_%s_const_srt' % (name, i))
        spinSrt = coreUtils.addChild(constSrt, 'group', name='%s_%s_spin_srt' % (name, i))

        bufferSrt = coreUtils.addChild(spinSrt, 'group', name='%s_%s_ctrlBufferSrt' % (name, i))
        negSrt = coreUtils.addChild(bufferSrt, 'group', name='%s_%s_xNegSrt' % (name, i))
        ctrl = controls.triCtrl(size=config.radius.get()*.2, aim=aimDict[i], name='%s_%s_ctrl' % (name, i))
        ctrl.setParent(negSrt)
        config.radius.connect(bufferSrt.tz)
        if i == 'T':
            config.height.connect(bufferSrt.ty)
        elif i == 'B':
            heightInvert = coreUtils.convert(config.height, -1, name='%s_heightInvert_utl' % name)
            heightInvert.output.connect(bufferSrt.ty)

        # Negate x translate and connect to spin
        conv = coreUtils.convert(ctrl.tx, -1, name='%s_%s_invertTX_utl' % (name, i))
        conv.output.connect(negSrt.tx)
        spinFac = coreUtils.convert(config.radius, 6.283, name='%s_%s_spinFactor_utl' % (name, i))
        spinAmount = coreUtils.divide(ctrl.tx, spinFac.output, name='%s_%s_spinAmount_utl' % (name, i))
        spinConv = coreUtils.convert(spinAmount.outputX, 6.283, name='%s_%s_spin_utl' % (name, i))

        if i == 'L' or i == 'R':
            spinOffset = pmc.createNode('animBlendNodeAdditiveDA', name='%s_%s_spinOffset_utl' % (name, i))
            spinConv.output.connect(spinOffset.inputA)
            config.spread.connect(spinOffset.inputB)
            spinOffset.output.connect(spinSrt.ry)
        else:
            spinConv.output.connect(spinSrt.ry)

        if i == 'R':
            constSrt.setParent(negScaleSrt)
            constSrt.s.set((1,1,1))
            constSrt.ry.set(0)
        ctrls.append(ctrl)

    # Distance nodes for auto scaling of tangents
    headSrt = coreUtils.decomposeMatrix(inputGrp.head_in_mtx, name='%s_head_in_mtx2Srt_utl' % name)
    dist_R_T = coreUtils.distanceBetweenNodes(ctrls[0], ctrls[1], name='%s_dist_R_T_utl' % name)
    scaledDist_R_T = coreUtils.divide(dist_R_T.distance, headSrt.outputScaleX, name='%s_scaledDist_R_T_utl' % name)

    dist_L_T = coreUtils.distanceBetweenNodes(ctrls[2], ctrls[1], name='%s_dist_L_T_utl' % name)
    scaledDist_L_T = coreUtils.divide(dist_L_T.distance, headSrt.outputScaleX, name='%s_scaledDist_L_T_utl' % name)

    dist_R_B = coreUtils.distanceBetweenNodes(ctrls[0], ctrls[3], name='%s_dist_R_B_utl' % name)
    scaledDist_R_B = coreUtils.divide(dist_R_B.distance, headSrt.outputScaleX, name='%s_scaledDist_R_B_utl' % name)

    dist_L_B = coreUtils.distanceBetweenNodes(ctrls[2], ctrls[3], name='%s_dist_L_B_utl' % name)
    scaledDist_L_B = coreUtils.divide(dist_L_B.distance, headSrt.outputScaleX, name='%s_scaledDist_L_B_utl' % name)


    # Build curves
    points = [(0,0,0) for i in range(7)]
    knots = [0,0,0,1,1,1,2,2,2]
    topCrv = pmc.curve(d=3, p=points, k=knots, name='%s_T_macro_crv' % name)
    btmCrv = pmc.curve(d=3, p=points, k=knots, name='%s_B_macro_crv' % name)

    p1 = coreUtils.pointMatrixMult((0,0,0), ctrls[0].worldMatrix[0], name='%s_R_pos_utl' % name)

    p2TanLen = coreUtils.convert(headSrt.outputScaleX, configDict['height'], name='%s_R_outTan_len_utl' % name)
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

    p8TanLen = coreUtils.convert(headSrt.outputScaleX, -configDict['height'], name='%s_L_outTan_len_utl' % name)
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

def buildTweaks(crv, root, tangents, numTweaks=7, name='mouth'):
    tweakCrv = curveUtils.curveBetweenPoints(start=(-10,0,0), end=(10,0,0), degree=2, numPoints=numTweaks, name='%s_T_tweak_crv' % name)
    # Create motions path nodes
    ctrls = []
    points=[]
    for i in range(numTweaks):
        num = str(i+1).zfill(2)
        mp = pmc.createNode('motionPath', name='%s_T_%s_tweakMotionPath_utl' % (name, num))
        crv.worldSpace[0].connect(mp.geometryPath)
        mp.uValue.set(1.0 / (numTweaks-1) * i)
        bfr = pmc.group(empty=1, name='%s_tweak_%s_bufferSrt' % (name, num))
        ctrl = controls.triControl(size = configDict['radius']*.15, aim='up', name='%s_tweak_%s_ctrl' % (name, num))
        ctrl.setParent(bfr)
        bfrLocalPos = coreUtils.pointMatrixMult(mp.allCoordinates, root.worldInverseMatrix[0], name='%s_tweakLocalPos_%s_utl' % (name, num)
        bfrLocalPos.output.connect(bfr.t)
        bfrAngle = pmc.creatNode('angleBetween', name='%s_tweakAngle_%s_utl' % (name, num))
        bfrLocalPos.outputX.connect(bfrAngle.input1X)
        bfrLocalPos.outputZ.connect(bfrAngle.input1Z)
        bfrAngle.input2.set((0,0,1))
        bfrAngle.eulerY.connect(bfr.ry)
        mp.fractionMode.set(1)
        ctrls.append(ctrl)
        d = coreUtils.decomposeMatrix(ctrl.worldMatrix[0], name='%s_tweak_%s_mtx2Srt_utl' % (name, num))
        points.append(d.outputTranslate)                                       
        if i==0:
            tanMult = coreUtils.convert(tangents[0].input1, 0.5, name=%s_T_tweakOutTan_utl' % name)
            p = coreUtils.pointMatrixMult(tanMult.output, ctrl.worldMatrix[0], name='%s_T_tweakOutTanPos_%s_utl' % name)
            points.append(p.output)                           
        elif i==(numTweaks-1):
            tanMult = coreUtils.convert(tangents[1].input1, 0.5, name=%s_T_tweakInTan_utl' % name)
            p = coreUtils.pointMatrixMult(tanMult.output, ctrl.worldMatrix[0], name='%s_T_tweakInTanPos_%s_utl' % name)
            points.append(p.output)
        # Make a degree 2 curve with points at each tweak location plus an extra point at each end which sits halfway
        # between the corner tweakers and the corner tangents
    







