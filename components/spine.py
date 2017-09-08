import pymel.core as pmc
import drRigging.utils.coreUtils as coreUtils
import drRigging.utils.curveUtils as curveUtils
import drRigging.objects.controls as controls
import drRigging.utils.componentUtils as componentUtils
import drRigging.components.base as drBase
reload(componentUtils)
reload(drBase)
reload(curveUtils)
reload(coreUtils)
reload(controls)

class DrSpine(drBase.DrBaseComponent):
    def __init__(self, name, joints, numJoints=None, axis='y', upAxis='z', worldUpAxis='z', cleanUp=1, reverse=1, ctrlInterval=2):
        '''

        :param name: name of new spine component e.g. 'spine_M'
        :param joints: guide joints from which to build spine
        :param numJoints: number of spine joints to expose for deformation
        :param axis: direction of spine
        :param upAxis: axis for motion path nodes to use as their upAxis
        :param worldUpAxis: axis for motion path nodes to align their upAxis to
        :param cleanUp: specifies whether to lock and hide relevant attributes and groups
        :param reverse: if true, a second fk chain will be created running top to bottom of the spine
        :param ctrlInterval: the number of slave transforms to put in between each fk control
        '''
        super(DrSpine, self).__init__(name)
        self.ctrls = []
        if not numJoints:
            numJoints=len(joints)
        self.build(joints, numJoints, axis, upAxis, worldUpAxis, reverse, ctrlInterval)
        if cleanUp:
            self.cleanUp()

    def build(self, joints, numJoints, axis, upAxis, worldUpAxis, reverse, ctrlInterval):

        # Create curves
        positions = [pmc.xform(joint, q=1, ws=1, t=1) for joint in joints]
        self.fkCrv = curveUtils.curveThroughPoints(positions=positions, name='%s_fkCrv' % self.name)
        self.fkCrv.setParent(self.rigGrp)

        self.ikCrv = curveUtils.curveThroughPoints(positions=positions, name='%s_ikCrv' % self.name)
        self.ikCrv.setParent(self.rigGrp)

        # controls
        ctrlSize = coreUtils.getDistance(joints[0], joints[-1]) * .5

        #base
        self.baseCtrl = controls.circleBumpCtrl(name='%s_base_ctrl' % self.name, axis=axis, radius=ctrlSize)
        self.baseCtrl.setParent(self.interfaceGrp)
        coreUtils.align(self.baseCtrl, joints[0])
        baseBuffer = coreUtils.addParent(self.baseCtrl, 'group', name='%s_base_buffer_srt' % self.name)
        self.ctrls.append(self.baseCtrl)

        self.baseSubCtrl = controls.circleBumpCtrl(name='%s_baseSub_ctrl' % self.name, axis=axis, radius=ctrlSize*.8)
        self.baseSubCtrl.setParent(self.baseCtrl)
        coreUtils.align(self.baseSubCtrl, joints[0])
        self.ctrls.append(self.baseSubCtrl)

        baseMtx = coreUtils.isDecomposed(self.baseSubCtrl)

        # settings
        self.settingsCtrl = controls.squareChamferCtrl(size=ctrlSize * .2, axis=axis,
                                                       name='%s_settings_ctrl' % self.name)
        self.settingsCtrl.setParent(self.baseSubCtrl)
        b = coreUtils.addParent(self.settingsCtrl, 'group', name='%s_settings_buffer_srt' % self.name)
        coreUtils.align(b, self.baseSubCtrl)
        pmc.setAttr('%s.t%s' % (b.name(), upAxis), ctrlSize * -1.5)
        self.ctrls.append(self.settingsCtrl)

        #FK
        self.fkCtrls = []
        for i in range(len(joints)-1):
            num = str(i+1).zfill(2)
            ctrlNum = str((i / ctrlInterval) + 1).zfill(2)
            c = None
            if i % ctrlInterval == 0:
                c = controls.circleBumpCtrl(name='%s_fk_%s_ctrl' % (self.name, ctrlNum), axis=axis, radius=ctrlSize*.5)
                self.ctrls.append(c)
            else:
                c = coreUtils.pmc.group(empty=1, name='%s_fk_%s_srt' % (self.name, num))
                self.ctrls[-1].t.connect(c.t)
                self.ctrls[-1].r.connect(c.r)
                self.ctrls[-1].s.connect(c.s)
            b = coreUtils.addParent(c, 'group', name='%s_fk%s_buffer_srt' % (self.name, num))
            coreUtils.align(b, joints[i])
            if i == 0:
                b.setParent(self.baseSubCtrl)
            else:
                b.setParent(self.fkCtrls[-1])
            self.fkCtrls.append(c)
        # reverse fk
        self.fkRevCtrls = []
        if reverse:
            for i in range(len(joints)-1):
                num = str(i+1).zfill(2)
                ctrlNum = str((i / ctrlInterval) + 1).zfill(2)
                c = None
                if i % ctrlInterval == 0:
                    c = controls.circleBumpCtrl(name='%s_fkReverse_%s_ctrl' % (self.name, ctrlNum), axis=axis, radius=ctrlSize*.4)
                    self.ctrls.append(c)
                else:
                    c = coreUtils.pmc.group(empty=1, name='%s_fkReverse_%s_srt' % (self.name, num))
                    self.ctrls[-1].t.connect(c.t)
                    self.ctrls[-1].r.connect(c.r)
                    self.ctrls[-1].s.connect(c.s)
                b = coreUtils.addParent(c, 'group', name='%s_fkReverse%s_buffer_srt' % (self.name, num))
                coreUtils.align(b, joints[len(joints) - i - 1])
                if i == 0:
                    b.setParent(self.fkCtrls[-1])
                else:
                    b.setParent(self.interfaceGrp)
                self.fkRevCtrls.append(c)

            srtList = []
            for i in range(len(self.fkRevCtrls)):
                num = str(i+1).zfill(2)
                fkCtrl = self.fkCtrls[len(self.fkCtrls)-i-1]
                revCtrl = self.fkRevCtrls[i]
                multMtx=None
                if i == 0:
                    multMtx = coreUtils.multiplyMatrices([fkCtrl.worldMatrix[0],
                                                          revCtrl.parentInverseMatrix[0],
                                                          revCtrl.worldMatrix[0]],
                                                         name='%s_localFkReverseMtx%s_utl' % (self.name, num))
                    d = coreUtils.decomposeMatrix(revCtrl.worldMatrix[0], name='%s_localFkReverseMtx01ToSrt_utl' % self.name)
                    revCtrl.rotateOrder.connect(d.inputRotateOrder)
                    d.outputTranslate.connect(self.fkCrv.controlPoints[len(joints)-1])
                    d.outputTranslate.connect(self.ikCrv.controlPoints[len(joints)-1])
                    srtList.append(d)
                elif i == len(self.fkRevCtrls) - 1:
                    multMtx = coreUtils.multiplyMatrices([self.baseSubCtrl.worldMatrix[0],
                                                          self.fkCtrls[len(self.fkCtrls)-i].worldInverseMatrix[0],
                                                          revCtrl.worldMatrix[0]],
                                                         name='%s_localFkReverseMtx%s_utl' % (self.name, num))
                else:
                    multMtx = coreUtils.multiplyMatrices([fkCtrl.worldMatrix[0],
                                                          self.fkCtrls[len(self.fkCtrls)-i].worldInverseMatrix[0],
                                                          revCtrl.worldMatrix[0]],
                                                         name='%s_localFkReverseMtx%s_utl' % (self.name, num))
                d = coreUtils.decomposeMatrix(multMtx.matrixSum, name='%s_localFkReverseMtx%sToSrt_utl' % (self.name, num))
                self.baseCtrl.rotateOrder.connect(d.inputRotateOrder)
                if i < len(self.fkRevCtrls)-1:
                    d.outputTranslate.connect(self.fkRevCtrls[i+1].getParent().t)
                    d.outputRotate.connect(self.fkRevCtrls[i+1].getParent().r)
                    d.outputScale.connect(self.fkRevCtrls[i+1].getParent().s)
                d.outputTranslate.connect(self.fkCrv.controlPoints[len(self.fkRevCtrls)-(i+1)])
                d.outputTranslate.connect(self.ikCrv.controlPoints[len(self.fkRevCtrls)-(i+1)])
                srtList.append(d)
        else:
            srtList = []
            d = coreUtils.decomposeMatrix(self.baseSubCtrl.worldMatrix[0], name='%s_baseMtxToSrt_utl' % self.name)
            self.baseSubCtrl.rotateOrder.connect(d.inputRotateOrder)
            d.outputTranslate.connect(self.fkCrv.controlPoints[0])
            d.outputTranslate.connect(self.ikCrv.controlPoints[0])
            srtList.append(d)
            for i in range(len(self.fkCtrls)):
                num = str(i+1).zfill(2)
                ctrl = self.fkCtrls[i]
                d = coreUtils.decomposeMatrix(ctrl.worldMatrix[0], name='%s_localFkMtx%sToSrt_utl' % (self.name, num))
                ctrl.rotateOrder.connect(d.inputRotateOrder)
                d.outputTranslate.connect(self.fkCrv.controlPoints[i+1])
                d.outputTranslate.connect(self.ikCrv.controlPoints[i+1])
                srtList.append(d)


        #IK
        # hip
        self.startCtrl = controls.squareCtrl(name='%s_start_ctrl' % self.name, axis=axis, size=ctrlSize)
        coreUtils.align(self.startCtrl, self.baseCtrl)
        startBuffer = coreUtils.addParent(self.startCtrl, 'group', name='%s_start_buffer_srt' % self.name)
        if reverse:
            startBuffer.setParent(self.interfaceGrp)
            self.ctrls.append(self.startCtrl)
            srtList[-1].outputTranslate.connect(startBuffer.t)
            srtList[-1].outputRotate.connect(startBuffer.r)
            srtList[-1].outputScale.connect(startBuffer.s)
        else:
            startBuffer.setParent(self.baseSubCtrl)

        # end
        self.endCtrl = controls.squareCtrl(name='%s_end_ctrl' % self.name, axis=axis, size=ctrlSize)
        coreUtils.align(self.endCtrl, joints[-1])
        endBuffer = coreUtils.addParent(self.endCtrl, 'group', name='%s_end_buffer_srt' % self.name)
        if reverse:
            endBuffer.setParent(self.interfaceGrp)
            self.ctrls.append(self.endCtrl)
            srtList[0].outputTranslate.connect(endBuffer.t)
            srtList[0].outputRotate.connect(endBuffer.r)
            srtList[0].outputScale.connect(endBuffer.s)
        else:
            endBuffer.setParent(self.fkCtrls[-1])
            d = coreUtils.decomposeMatrix(endBuffer.worldMatrix[0], name='%s_endMtxToSrt_utl' % self.name)
            self.endCtrl.rotateOrder.connect(d.inputRotateOrder)
            d.outputTranslate.connect(self.fkCrv.controlPoints[len(joints)-1], f=1)
            d.outputTranslate.connect(self.ikCrv.controlPoints[len(joints)-1], f=1)
            srtList.append(d)


        # rotate orders for ctrls
        rotateOrderDict={'x':0, 'y':1, 'z':2}
        for ctrl in self.ctrls:
            ctrl.rotateOrder.set(rotateOrderDict[axis])


        # crvInfo
        crvInfo = pmc.createNode('curveInfo', name='%s_ikCrvInfo_utl' % self.name)
        crvShape = coreUtils.getShape(self.fkCrv)
        crvShape.worldSpace[0].connect(crvInfo.inputCurve)

        # stretch
        pmc.addAttr(self.mainGrp, longName='stretch', at='double', k=0, h=1)
        restLenMD = coreUtils.multiply(crvInfo.arcLength.get(), baseMtx.outputScaleX, name='%s_restLength_utl' % self.name)
        stretchMD = coreUtils.divide(restLenMD.outputX, crvInfo.arcLength, name='%s_stretchFactor_utl' % self.name)
        stretchMD.outputX.connect(self.mainGrp.stretch)

        # ik curve skinning
        endGuideJoint = coreUtils.addChild(self.rigGrp, 'joint', '%s_endGuide_jnt' % self.name)
        endJointBuffer = coreUtils.addParent(endGuideJoint, 'group', name='%s_endJnt_buffer_srt' % self.name)
        endJoint = coreUtils.addChild(endJointBuffer, 'joint', '%s_end_jnt' % self.name)
        srt = srtList[-1]
        if reverse:
            srt = srtList[0]
        srt.outputTranslate.connect(endJointBuffer.t)
        srt.outputRotate.connect(endJointBuffer.r)
        srt.outputScale.connect(endJointBuffer.s)
        self.endCtrl.t.connect(endGuideJoint.t)
        self.endCtrl.r.connect(endGuideJoint.r)
        self.endCtrl.t.connect(endJoint.t)
        self.endCtrl.r.connect(endJoint.r)

        startGuideJoint = coreUtils.addChild(self.rigGrp, 'joint', '%s_startGuide_jnt' % self.name)
        startJointBuffer = coreUtils.addParent(startGuideJoint, 'group', name='%s_startJnt_buffer_srt' % self.name)
        startJoint = coreUtils.addChild(startJointBuffer, 'joint', '%s_start_jnt' % self.name)
        srt = srtList[-1]
        if not reverse:
            srt = srtList[0]
        srt.outputTranslate.connect(startJointBuffer.t)
        srt.outputRotate.connect(startJointBuffer.r)
        srt.outputScale.connect(startJointBuffer.s)
        self.startCtrl.t.connect(startGuideJoint.t)
        self.startCtrl.r.connect(startGuideJoint.r)
        self.startCtrl.t.connect(startJoint.t)
        self.startCtrl.r.connect(startJoint.r)

        jointScaleMD = coreUtils.divide(1.0, self.mainGrp.stretch, name='%s_ikJointScale_utl' % self.name)

        coreUtils.connectAttrToMany(jointScaleMD.outputX, [startJoint.sx, startJoint.sy, startJoint.sz,
                                                               endJoint.sx, endJoint.sy, endJoint.sz])

        guideSkin = pmc.skinCluster(endGuideJoint, startGuideJoint, self.fkCrv, dr=1)
        endJointBuffer.worldInverseMatrix[0].connect(guideSkin.bindPreMatrix[0])
        startJointBuffer.worldInverseMatrix[0].connect(guideSkin.bindPreMatrix[1])

        skin = pmc.skinCluster(endJoint, startJoint, self.ikCrv, dr=2)
        endJointBuffer.worldInverseMatrix[0].connect(skin.bindPreMatrix[0])
        startJointBuffer.worldInverseMatrix[0].connect(skin.bindPreMatrix[1])

        # create attribute on settings control to choose upAxis and avoid flipping
        enumString = ''
        axes = ['x', 'y', 'z']
        for a in axes:
            if a != axis:
                enumString += (a + ':')
        pmc.addAttr(self.settingsCtrl, ln='upAxis', at='enum', enumName=enumString, k=1, h=0)
        upVecCond = pmc.createNode('condition', name='%s_upVec_utl' % self.name)
        self.settingsCtrl.upAxis.connect(upVecCond.firstTerm)
        if axis=='x':
            upVecCond.colorIfTrue.set((0, 1, 0))
        else:
            upVecCond.colorIfTrue.set((1, 0, 0))
        if axis=='y':
            upVecCond.colorIfFalse.set((0, 0, 1))
        else:
            upVecCond.colorIfFalse.set((0, 1, 0))

        upAxisCond = pmc.createNode('condition', name='%s_upAxis_utl' % self.name)
        self.settingsCtrl.upAxis.connect(upAxisCond.firstTerm)
        if axis == 'x':
            upAxisCond.colorIfTrueR.set(1)
        else:
            upAxisCond.colorIfTrueR.set(0)
        if axis == 'y':
            upAxisCond.colorIfFalseR.set(2)
        else:
            upAxisCond.colorIfFalseR.set(1)

        # pathNodes
        mps = curveUtils.nodesAlongCurve(crv=self.ikCrv, numNodes=numJoints, name=self.name, followAxis=axis,
                                         upNode=self.baseSubCtrl, upAxis=upAxis, upVec=worldUpAxis)
        for i in range(len(mps['mpNodes'])):
            mp = mps['mpNodes'][i]
            upVecCond.outColor.connect(mp.worldUpVector)
            upAxisCond.outColorR.connect(mp.upAxis)

            if i < numJoints/2:
                self.startCtrl.worldMatrix[0].connect(mp.worldUpMatrix, f=1)
            else:
                self.endCtrl.worldMatrix[0].connect(mp.worldUpMatrix, f=1)
            self.baseCtrl.rotateOrder.connect(mp.rotateOrder)
        for grp in mps['grps']:
            grp.setParent(self.rigGrp)
            self.baseCtrl.rotateOrder.connect(grp.rotateOrder)

        # twisting
        self.twistReader = coreUtils.isolateTwist(mps['grps'][(numJoints / 2)].worldMatrix[0],
                                                  mps['grps'][(numJoints / 2)-1].worldInverseMatrix[0],
                                                  name=self.name, axis=axis)
        twistAttr = pmc.Attribute('%s.outputRotate%s' % (self.twistReader[2].name(), axis.upper()))
        twistSrtList = []
        for i in range(numJoints / 2):
            mp = mps['mpNodes'][i]
            grp = mps['grps'][i]
            g = coreUtils.addChild(grp, 'group', grp.name().replace('grp', 'twist_srt'))
            self.baseCtrl.rotateOrder.connect(g.rotateOrder)
            baseMtx.outputScale.connect(g.s)
            uc = coreUtils.convert(twistAttr, (1.0 / (numJoints-1))*i, name=g.name().replace('_srt', '_utl'))
            uc.output.connect(pmc.Attribute('%s.r%s' % (g.name(), axis)))
            twistSrtList.append(g)
        for i in range(numJoints - (numJoints / 2)):
            index = i + (numJoints / 2)
            mp = mps['mpNodes'][index]
            grp = mps['grps'][index]
            g = coreUtils.addChild(grp, 'group', grp.name().replace('grp', 'twist_srt'))
            self.baseCtrl.rotateOrder.connect(g.rotateOrder)
            baseMtx.outputScale.connect(g.s)
            uc = coreUtils.convert(twistAttr, (-1.0 / (numJoints-1))*((numJoints - (numJoints / 2)-i)-1), name=g.name().replace('_srt', '_utl'))
            uc.output.connect(pmc.Attribute('%s.r%s' % (g.name(), axis)))
            twistSrtList.append(g)

        # squetch
        for i in range(numJoints):
            attr = pmc.addAttr(self.settingsCtrl, longName='squetch_%s_amount' % str(i+1).zfill(2), at='double', k=1, h=0)
            blend = coreUtils.blend(self.mainGrp.stretch, 1.0, name='blend_%s_squetch_%s_UTL' % (self.name, str(i+1).zfill(2)),
                                    blendAttr=self.settingsCtrl.attr('squetch_%s_amount' % str(i+1).zfill(2)))
            scaleDict = {'x':mps['grps'][i].sx, 'y':mps['grps'][i].sy, 'z':mps['grps'][i].sz}
            for key in scaleDict.keys():
                if key != axis:
                    blend.outputR.connect(scaleDict[key])

        # expose joints
        for i in range(numJoints):
            componentUtils.exposeDeformation(twistSrtList[i], '%s_%s' % (self.name, str(i+1).zfill(2)))

        # colorize
        coreUtils.colorize('green', self.fkCtrls + [self.baseCtrl, self.settingsCtrl])
        coreUtils.colorize('yellow', self.fkRevCtrls + [self.startCtrl, self.endCtrl, self.baseSubCtrl])

    def cleanUp(self):
        super(DrSpine, self).cleanUp()
        fkCtrls = [ctrl for ctrl in self.fkCtrls] + [ctrl for ctrl in self.fkRevCtrls]
        coreUtils.attrCtrl(nodeList=fkCtrls, attrList=['tx', 'ty', 'tz'])
        coreUtils.attrCtrl(nodeList=[c for c in self.ctrls if c!=self.baseCtrl and c!=self.baseSubCtrl],
                           attrList=['sx', 'sy', 'sz'])
        coreUtils.attrCtrl(nodeList=[self.settingsCtrl], attrList=['tx', 'ty', 'tz', 'rx', 'ry', 'rz', 'sx', 'sy', 'sz'])
