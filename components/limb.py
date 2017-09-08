import drRigging.components.base as drBase
import drRigging.components.twistySegment as drTwistySegment
import drRigging.utils.coreUtils as coreUtils
import drRigging.utils.componentUtils as componentUtils
import drRigging.objects.controls as controls
import drRigging.objects.rigObjects as rigObjects
import pymel.core as pmc
reload(drBase)
reload(controls)
reload(rigObjects)
reload(coreUtils)
reload(componentUtils)
reload(drTwistySegment)

class DrLimb(drBase.DrBaseComponent):
    '''
    builds an ik fk limb based on 4 input joints. Joints should be oriented along the x axis.
    bending occurs toward the positive z axis
    '''
    def __init__(self, name, joints, alignIkToJointsUpAxis=0, cleanUp=1):
        super(DrLimb, self).__init__(name)
        self.build(joints, alignIkToJointsUpAxis)
        if cleanUp:
            self.cleanUp()

    def build(self, joints, alignIkToJointsUpAxis):

        # duplicate joints
        self.resultJoints = coreUtils.duplicateJoints(joints, name='%s_result_XX_jnt' % self.name)
        self.fkJoints = coreUtils.duplicateJoints(joints, name='%s_fk_XX_jnt' % self.name)
        self.ikJoints = coreUtils.duplicateJoints(joints, name='%s_ik_XX_jnt' % self.name)
        self.jointBufferGrp = coreUtils.addChild(self.rigGrp, 'group', '%s_jointsBuffer_srt' % self.name)
        coreUtils.align(self.jointBufferGrp, self.resultJoints[0])
        self.resultJoints[0].setParent(self.jointBufferGrp)
        self.fkJoints[0].setParent(self.jointBufferGrp)
        self.ikJoints[0].setParent(self.jointBufferGrp)

        ctrlSize = coreUtils.getDistance(self.resultJoints[0], self.resultJoints[1]) * .25

        # Orientation for controls and aim constraints
        axis = 'x'
        aimVec = (1, 0, 0)
        if self.resultJoints[1].tx.get() < 0.0:
            axis = '-x'
            aimVec = (-1, 0, 0)

        self.settingsCtrl = controls.squareChamferCtrl(name='%s_settings_ctrl' % self.name, size=ctrlSize*.33)
        self.settingsCtrl.setParent(self.interfaceGrp)
        pmc.addAttr(self.settingsCtrl, ln='ik_fk_blend', at='double', k=1, h=0, maxValue=1.0, minValue=0.0)
        settingsBuffer = coreUtils.addParent(self.settingsCtrl, 'group', '%s_settings_buffer_srt' % self.name)
        settingsOffset = ctrlSize * 2
        if axis == '-x':
            settingsOffset *= -1
        self.settingsCtrl.ty.set(settingsOffset)
        endSrt = coreUtils.decomposeMatrix(self.resultJoints[2].worldMatrix[0],
                                           name='%s_endWorldMtxToSrt_utl' % self.name)
        endSrt.outputTranslate.connect(settingsBuffer.t)
        endSrt.outputRotate.connect(settingsBuffer.r)
        endSrt.outputScale.connect(settingsBuffer.s)
        self.ctrls.append(self.settingsCtrl)

        # Create a dummy input to connect parts of the limb to
        self.baseInput = coreUtils.addChild(self.inputGrp, 'group', name='%s_base_in_srt' % self.name)
        coreUtils.align(self.baseInput, joints[0])
        baseMtx = coreUtils.isDecomposed(self.baseInput)

        # set up blending
        for rj, ij, fj in zip(self.resultJoints, self.ikJoints, self.fkJoints):
            coreUtils.ikFkBlend(ij, fj, rj, blendAttr='%s.ik_fk_blend' % self.settingsCtrl)

        # fk controls
        self.fkCtrlsGrp = coreUtils.addChild(self.interfaceGrp, 'group', '%s_self.fkCtrls_hrc' % self.name)
        self.settingsCtrl.ik_fk_blend.connect(self.fkCtrlsGrp.visibility)
        buffer = None
        self.fkCtrls = []
        for i in range(len(self.fkJoints)-1):
            fj = self.fkJoints[i]
            num = str(i+1).zfill(2)
            c = controls.circleBumpCtrl(name='%s_fk_%s_ctrl' % (self.name, num), axis=axis, radius=ctrlSize)
            coreUtils.align(c, fj)
            buffer = coreUtils.addParent(c, 'group', '%s_fk%s_buffer_srt' % (self.name, num))
            if i == 0:
                buffer.setParent(self.fkCtrlsGrp)
                coreUtils.connectAttrs(c, fj, ['t', 'r', 's'])
            else:
                buffer.setParent(self.fkCtrls[-1])
                m = coreUtils.decomposeLocalMatrix(c, depth=1)
                m.outputTranslate.connect(fj.t)
                m.outputScale.connect(fj.s)
                c.rotate.connect(fj.r)
            self.fkCtrls.append(c)
            self.ctrls.append(c)

        # rotate order
        for node in [self.fkCtrls[1], self.fkJoints[1], self.ikJoints[1], self.resultJoints[1]]:
            node.rotateOrder.set(3)

        # ik controls
        self.ikCtrl = controls.boxCtrl(name='%s_ik_ctrl' % self.name, size=ctrlSize)
        if alignIkToJointsUpAxis:
            aimAxis = 'x'
            if pmc.xform(joints[2], q=1, t=1, ws=1) > pmc.xform(joints[3], q=1, t=1, ws=1):
                aimAxis = '-x'
            elif pmc.xform(joints[2], q=1, t=1, ws=1) < pmc.xform(joints[3], q=1, t=1, ws=1):
                aimAxis = 'x'
            pmc.xform(self.ikCtrl, ws=1, m=coreUtils.getAimMatrix(start=self.resultJoints[2],
                                                                  end=self.resultJoints[3],
                                                                  axis=aimAxis, upAxis='y',
                                                                  worldUpAxis=alignIkToJointsUpAxis,
                                                                  upNode=self.resultJoints[2]))
        else:
            coreUtils.align(self.ikCtrl, self.ikJoints[2], orient=0)
        self.ikCtrlsGrp = coreUtils.addChild(self.interfaceGrp, 'group', '%s_ikCtrls_hrc' % self.name)
        self.ikCtrl.setParent(self.ikCtrlsGrp)
        ikBuffer = coreUtils.addParent(self.ikCtrl, 'group', '%s_ik_buffer_srt' % self.name)
        ikFkRev = coreUtils.reverse(self.settingsCtrl.ik_fk_blend, '%s_ikFkBlendReverse_UTL' % self.name)
        ikFkRev.outputX.connect(self.ikCtrlsGrp.visibility)
        self.ctrls.append(self.ikCtrl)

        pmc.addAttr(self.ikCtrl, longName='stretch', at='double', minValue=0, maxValue=1, defaultValue=0, keyable=True)
        pmc.addAttr(self.ikCtrl, longName='twist', at='double', keyable=True)
        pmc.addAttr(self.ikCtrl, longName='soft', at='double', minValue=0, defaultValue=0, keyable=True)
        pmc.addAttr(self.ikCtrl, longName='mid_pin', at='double', keyable=True, minValue=0, maxValue=1)
        pmc.addAttr(self.ikCtrl, longName='force_straight', at='double', keyable=True, h=0, minValue=0.0, maxValue=1.0)

        # Soft non-stretchy IK stuff
        ik_grp = coreUtils.addChild(self.rigGrp, 'group', name='%s_ik_hrc' % self.name)
        self.ikBufferGrp = coreUtils.addChild(ik_grp, 'group', name='%s_ik_srt' % self.name)
        coreUtils.align(self.ikBufferGrp, self.jointBufferGrp)

        self.softBlend_loc = coreUtils.createAlignedNode(self.ikCtrl, 'locator', name='%s_ik_softBlend_loc' % self.name)
        self.softBlend_loc.setParent(ik_grp)

        self.ctrl_loc = coreUtils.createAlignedNode(self.ikCtrl, 'locator', name='%s_ikCtrl_loc' % self.name)
        self.ctrl_loc.setParent(ik_grp)
        pmc.parentConstraint(self.ikCtrl, self.ctrl_loc)

        aim_loc = coreUtils.addChild(self.ikBufferGrp, 'locator', name='%s_softIkaim_loc' % self.name)
        pmc.aimConstraint(self.ctrl_loc, aim_loc, upVector=(0, 0, 0))

        btm_loc = coreUtils.createAlignedNode(self.ikCtrl, 'locator', name='%s_ikBtm_loc' % self.name)
        btm_loc.setParent(aim_loc)

        chainLen = abs(self.ikJoints[1].tx.get() + self.ikJoints[2].tx.get())

        ctrlDist = coreUtils.distanceBetweenNodes(aim_loc, self.ctrl_loc, name='%s_ikCtrl_utl' % self.name)
        globalScaleDiv = coreUtils.divide(1.0, baseMtx.outputScaleX, name='%s_globalScaleDiv_utl' % self.name)
        isStretchedMult = coreUtils.multiply(ctrlDist.distance, globalScaleDiv.outputX,
                                             name='%s_isStretched_utl' % self.name)

        softDist = coreUtils.distanceBetweenNodes(btm_loc, self.softBlend_loc, name='%s_soft_utl' % self.name)
        stretchDist = coreUtils.distanceBetweenNodes(aim_loc, self.softBlend_loc, name='%s_stretch_utl' % self.name)

        chainLenMinusSoft = coreUtils.minus([chainLen, self.ikCtrl.soft],
                                            name='%s_chainLenMinusSoft_utl' % self.name)

        isStretchedCond = pmc.createNode('condition', name='%s_isStretched_utl' % self.name)
        isStretchedCond.operation.set(2)
        isStretchedMult.outputX.connect(isStretchedCond.firstTerm)
        chainLenMinusSoft.output1D.connect(isStretchedCond.secondTerm)
        isStretchedMult.outputX.connect(isStretchedCond.colorIfFalseR)

        isSoftCond = pmc.createNode('condition', name='%s_isSoft_utl' % self.name)
        isSoftCond.operation.set(2)
        self.ikCtrl.soft.connect(isSoftCond.firstTerm)
        isSoftCond.colorIfFalseR.set(chainLen)

        ctrlDistMinusSoftChain = coreUtils.minus([isStretchedMult.outputX, chainLenMinusSoft.output1D],
                                                 name='%s_ctrlDistMinusSoftChain_utl' % self.name)

        divideBySoft = coreUtils.safeDivide(ctrlDistMinusSoftChain.output1D, self.ikCtrl.soft,
                                            name='%s_divideBySoft_utl' % self.name)

        invert = coreUtils.multiply(divideBySoft.outputX, -1, name='%s_invertSoft_utl' % self.name)

        exp = coreUtils.power(2.718282, invert.outputX, name='%s_exponential_utl' % self.name)

        multiplyBySoft = coreUtils.multiply(exp.outputX, self.ikCtrl.soft, name='%s_multiplyBySoft_utl' % self.name)

        minusFromChainLen = coreUtils.minus([chainLen, multiplyBySoft.outputX],
                                            name='%s_minusFromChainLen_utl' % self.name)

        minusFromChainLen.output1D.connect(isSoftCond.colorIfTrueR)

        isSoftCond.outColorR.connect(isStretchedCond.colorIfTrueR)

        isStretchedCond.outColorR.connect(btm_loc.tx)

        # IK Solvers
        ikHandleGrp = coreUtils.createAlignedNode(self.ikCtrl, 'group', name='%s_ikHandle_srt' % self.name)
        ikHandleGrp.setParent(self.softBlend_loc)
        pmc.orientConstraint(self.ikCtrl, ikHandleGrp, mo=1)

        ikHandle = pmc.ikHandle(solver='ikRPsolver',
                                name='%s_ikHandle' % self.name,
                                startJoint=self.ikJoints[0],
                                endEffector=self.ikJoints[2],
                                setupForRPsolver=1)[0]
        ikHandle.setParent(ikHandleGrp)
        self.ikCtrl.twist.connect(ikHandle.twist)

        endIkHandle = pmc.ikHandle(solver='ikSCsolver',
                                   name='%s_end_ikHandle' % self.name,
                                   startJoint=self.ikJoints[2],
                                   endEffector=self.ikJoints[3],
                                   setupForRPsolver=1)[0]
        endIkHandle.setParent(ikHandleGrp)

        # Pole Vector
        pvAimGrp = coreUtils.addChild(self.ikBufferGrp, 'group', name='%s_pvAim_srt' % self.name)
        pmc.aimConstraint(btm_loc, pvAimGrp, mo=0, u=(0, 0, 0), aimVector=aimVec)
        self.pvCtrl = controls.crossCtrl(name='%s_pv_ctrl' % self.name, size=ctrlSize)
        poleOffset = 3
        if '-' in axis:
            poleOffset = -3
        self.pvCtrl.setParent(self.ikJoints[1])
        self.pvCtrl.t.set((0, 0, ctrlSize * poleOffset))
        self.pvCtrl.setParent(self.ikCtrlsGrp)
        self.pvBufferGrp = coreUtils.addParent(self.pvCtrl, 'group', '%s_pv_buffer_srt' % self.name)
        pmc.poleVectorConstraint(self.pvCtrl, ikHandle)
        self.ctrls.append(self.pvCtrl)

        # stretchy soft IK stuff

        pc = pmc.pointConstraint(btm_loc, self.ctrl_loc, self.softBlend_loc)
        stretchRev = coreUtils.reverse(self.ikCtrl.stretch, name='%s_stretchReverse_utl' % self.name)
        stretchRev.outputX.connect('%s.%sW0' % (pc.nodeName(), btm_loc.nodeName()))
        self.ikCtrl.stretch.connect('%s.%sW1' % (pc.nodeName(), self.ctrl_loc.nodeName()))

        scaledSoftDist = coreUtils.multiply(globalScaleDiv.outputX, softDist.distance,
                                            name='%s_scaledSoftDist_utl' % self.name)

        # Stretchy Mid
        midLen = coreUtils.multiply((self.ikJoints[1].tx.get() / chainLen), scaledSoftDist.outputX,
                                    name='%s_midLen_utl' % self.name)

        stretchMidLen = coreUtils.multiply(self.ikCtrl.stretch, midLen.outputX,
                                           name='%s_stretchMidLen_utl' % self.name)

        stretchMidLenPlusBoneLen = coreUtils.add([self.ikJoints[1].tx.get(), stretchMidLen.outputX],
                                                 name='%s_stretchMidLenPlusBoneLen_utl' % self.name)

        pinUpperDist = coreUtils.distanceBetweenNodes(self.ikBufferGrp, self.pvCtrl,
                                                      name='%s_pinUpper_utl' % self.name)
        pinMult = 1.0
        if '-' in axis:
            pinMult = -1.0
        pinUpper_uc = coreUtils.convert(pinUpperDist.distance, pinMult, name='%s_pinUpperDist_utk' % self.name)
        pinUpperGlobalScale = coreUtils.multiply(globalScaleDiv.outputX, pinUpper_uc.output,
                                                 name='%s_pinUpperGlobalScale_utl' % self.name)
        pinUpperBlend = coreUtils.blend(input1=pinUpperGlobalScale.outputX, input2=stretchMidLenPlusBoneLen.output1D,
                                        blendAttr=self.ikCtrl.mid_pin, name='%s_pinUpper_utl' % self.name)

        # Stretchy Bot
        botLen = coreUtils.multiply((self.ikJoints[2].tx.get() / chainLen), scaledSoftDist.outputX,
                                    name='%s_botLen_utl' % self.name)

        stretchBotLen = coreUtils.multiply(self.ikCtrl.stretch, botLen.outputX,
                                           name='%s_stretchBotLen_utl' % self.name)

        stretchBotLenPlusBoneLen = coreUtils.add([self.ikJoints[2].tx.get(), stretchBotLen.outputX],
                                                 name='%s_stretchBotLenPlusBoneLen_utl' % self.name)

        pinLowerDist = coreUtils.distanceBetweenNodes(self.softBlend_loc, self.pvCtrl,
                                                      name='%s_pinLower_utl' % self.name)
        pinLower_uc = coreUtils.convert(pinLowerDist.distance, pinMult, name='%s_pinLowerDist_utl' % self.name)
        pinLowerGlobalScale = coreUtils.multiply(globalScaleDiv.outputX, pinLower_uc.output,
                                                 name='%s_pinLowerGlobalScale_utl' % self.name)
        pinLowerBlend = coreUtils.blend(input1=pinLowerGlobalScale.outputX, input2=stretchBotLenPlusBoneLen.output1D,
                                        blendAttr=self.ikCtrl.mid_pin, name='%s_pinLower_utl' % self.name)

        # Add ability to force straight arm
        straightUpper_md = coreUtils.multiply(stretchDist.distance, self.ikJoints[1].tx.get() / chainLen,
                                              name='%s_midLenTimesChainLen_utl' % self.name)
        straightLower_md = coreUtils.multiply(stretchDist.distance, self.ikJoints[2].tx.get() / chainLen,
                                              name='%s_botLenTimesChainLen_utl' % self.name)
        straightUpperBlend = coreUtils.blend(input1=straightUpper_md.outputX, input2=pinUpperBlend.outputR,
                                             blendAttr=self.ikCtrl.force_straight,
                                             name='%s_straightUpper_utl' % self.name)
        straightLowerBlend = coreUtils.blend(input1=straightLower_md.outputX, input2=pinLowerBlend.outputR,
                                             blendAttr=self.ikCtrl.force_straight,
                                             name='%s_straightLower_utl' % self.name)
        straightUpperBlend.outputR.connect(self.ikJoints[1].tx)
        straightLowerBlend.outputR.connect(self.ikJoints[2].tx)

        # connect to base input
        d = coreUtils.isDecomposed(self.baseInput)
        coreUtils.connectDecomposedMatrix(d, self.jointBufferGrp)
        coreUtils.connectDecomposedMatrix(d, self.ikBufferGrp)

        # Add basic space switching to ik ctrl, pole vector and top fk ctrl - additional spaces can be added at any point
        pvTargets = componentUtils.addSpaceSwitch(self.pvBufferGrp, 'pv', ['limb'], type='parent',
                                                  ctrlNode=self.pvCtrl, targetNodes=[pvAimGrp])
        ikTargets = componentUtils.addSpaceSwitch(ikBuffer, 'ik', ['limb'], type='parent',
                                                  ctrlNode=self.ikCtrl, targetNodes=[self.baseInput])
        fkTargets = componentUtils.addSpaceSwitch(self.fkCtrls[0].getParent(), 'fk', ['limb'], type='rotate',
                                                  ctrlNode=self.fkCtrls[0], targetNodes=[self.baseInput])
        d.outputTranslate.connect(self.fkCtrls[0].getParent().t)
        d.outputScale.connect(self.fkCtrls[0].getParent().s)


    def cleanUp(self):
        super(DrLimb, self).cleanUp()
        coreUtils.attrCtrl(nodeList=self.fkCtrls, attrList=['ty', 'tz', 'sx', 'sy', 'sz', 'visibility'])
        coreUtils.attrCtrl(nodeList=[self.ikCtrl], attrList=['sx', 'sy', 'sz', 'visibility'])
        coreUtils.attrCtrl(nodeList=[self.pvCtrl], attrList=['rx', 'ry', 'rz', 'sx', 'sy', 'sz', 'visibility'])
        coreUtils.attrCtrl(nodeList=[self.settingsCtrl], attrList=['tx', 'ty', 'tz','rx', 'ry', 'rz', 'sx', 'sy', 'sz', 'visibility'])

class DrTwistyLimb(DrLimb):
    '''
        builds an ik fk limb based on 4 input joints. Joints should be oriented along the x axis.
        bending occurs toward the positive z axis
        Adds twisty segments for upper and lower limb and adds an average ctrl at the middle joint
    '''

    def __init__(self, name, joints, alignIkToJointsUpAxis=0, cleanUp=1, numTwistJoints=4):
        super(DrTwistyLimb, self).__init__(name, joints, alignIkToJointsUpAxis, cleanUp)
        self.buildTwistyLimb(numTwistJoints)
        if cleanUp:
            self.cleanUp()

    def buildTwistyLimb(self, numTwistJoints):
        ctrlSize = coreUtils.getDistance(self.resultJoints[0], self.resultJoints[1]) * .25
        # Add mid ctrl
        self.midBuffer = rigObjects.orientationAverageNode(self.resultJoints[0], self.resultJoints[1], '%s_mid_buffer' % self.name)
        self.midCtrl = controls.squareCtrl(size=ctrlSize, axis='x', name='%s_mid_ctrl' % self.name)
        coreUtils.align(self.midCtrl, self.midBuffer)
        self.midCtrl.setParent(self.midBuffer)
        self.midBuffer.setParent(self.interfaceGrp)
        pmc.addAttr(self.midCtrl, ln='auto_bend', at='double', k=1, h=0, minValue=0.0, maxValue=1.0)
        self.ctrls.append(self.midCtrl)

        # Create twisty segments
        self.upperTwist = drTwistySegment.DrTwistySegment(self.fkCtrls[0], self.midCtrl,
                                                     name='%sUpper_%s' % (self.name.split('_')[0],
                                                                          self.name.split('_')[1]),
                                                     numDivisions=numTwistJoints)
        self.upperTwist.mainGrp.setParent(self.subGrp)

        self.lowerTwist = drTwistySegment.DrTwistySegment(self.midCtrl, self.fkCtrls[2],
                                                     name='%sLower_%s' % (self.name.split('_')[0],
                                                                          self.name.split('_')[1]),
                                                     numDivisions=numTwistJoints)
        self.lowerTwist.mainGrp.setParent(self.subGrp)

        if self.resultJoints[1].tx.get() > 0.0:
            self.lowerTwist.mainGrp.invertTwist.set(1)
            self.upperTwist.mainGrp.invertTwist.set(1)

        upperNonRoll = rigObjects.nonTwistingChild(self.resultJoints[0], name='%s_upper' % self.name)
        componentUtils.connectIO(self.upperTwist.startIn, upperNonRoll, '%s_upperStart' % self.name)
        componentUtils.connectIO(self.upperTwist.endIn, self.midCtrl, '%s_upperEnd' % self.name)

        componentUtils.connectIO(self.lowerTwist.startIn, self.midCtrl, '%s_lowerStart' % self.name)
        componentUtils.connectIO(self.lowerTwist.endIn, self.resultJoints[2], '%s_lowerEnd' % self.name)

        self.midCtrl.auto_bend.connect(self.upperTwist.mainGrp.endAuto)
        self.midCtrl.auto_bend.connect(self.lowerTwist.mainGrp.startAuto)

        pmc.addAttr(self.settingsCtrl, ln='bendyCtrls_vis', at='bool', k=0, h=0)
        pmc.setAttr(self.settingsCtrl.bendyCtrls_vis, cb=1)
        self.settingsCtrl.bendyCtrls_vis.connect(self.upperTwist.mainGrp.visibility)
        self.settingsCtrl.bendyCtrls_vis.connect(self.lowerTwist.mainGrp.visibility)

    def cleanUp(self):
        super(DrTwistyLimb, self).cleanUp()