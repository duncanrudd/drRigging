import drRigging.components.base as drBase
reload(drBase)
import drRigging.utils.coreUtils as coreUtils
reload(coreUtils)
import drRigging.utils.curveUtils as curveUtils
reload(curveUtils)
import drRigging.objects.controls as controls
reload(controls)
import pymel.core as pmc
import drRigging.utils.componentUtils as componentUtils
reload(componentUtils)

class DrSnake(drBase.DrBaseComponent):
    '''

    '''
    def __init__(self, start, end, name, numFkCtrls, jointsPerCtrl, numIkCtrls, latticeDivisions=4, axis='x', upAxis='y',  cleanUp=1):
        super(DrSnake, self).__init__(name)

        self.build(start, end, numFkCtrls, jointsPerCtrl, numIkCtrls, latticeDivisions, axis, upAxis)

        if cleanUp:
            self.cleanUp()

    def build(self, start, end, numFkCtrls, jointsPerCtrl, numIkCtrls, latticeDivisions, axis, upAxis):
        start, end = coreUtils.getStartAndEnd(start, end)
        ctrlSize = coreUtils.getDistance(start, end) * .2

        # main control
        baseCtrl = controls.circleBumpCtrl(radius=ctrlSize,
                                           name='%s_base_ctrl' % self.name, axis=axis)
        baseSrt = coreUtils.decomposeMatrix(baseCtrl.worldMatrix[0], name='%s_baseWorldMtxToSrt_utl' % baseCtrl)
        baseBuffer = coreUtils.addParent(baseCtrl, 'group', name='%s_base_buffer_srt' % self.name)
        baseBuffer.setParent(self.interfaceGrp)
        self.ctrls.append(baseCtrl)

        pmc.addAttr(baseCtrl, ln='stretch', at='double', k=1, h=0, minValue=0.0, maxValue=1.0)
        pmc.addAttr(baseCtrl, ln='anchor', at='double', k=1, h=0, minValue=0.0, maxValue=1.0)
        pmc.addAttr(baseCtrl, ln='head_extend', at='double', k=1, h=0, minValue=0.0, maxValue=1.0)
        pmc.addAttr(baseCtrl, ln='tail_extend', at='double', k=1, h=0, minValue=0.0, maxValue=1.0)

        # Build ik curves
        ikSoftCrv = curveUtils.curveBetweenNodes(start, end, numCVs=numIkCtrls, name='%s_ik_soft_crv' % self.name)
        ikSoftCrv.setParent(self.rigGrp)

        ikHardCrv = curveUtils.curveBetweenNodes(start, end, numCVs=(numIkCtrls*3)-2, name='%s_ik_hard_crv' % self.name)
        ikHardCrv.setParent(self.rigGrp)

        # ik controls
        cvs = ikSoftCrv.getCVs(space='world')
        self.ikCtrls = []
        for i in range(numIkCtrls):
            num = str(i+1).zfill(2)
            c = controls.ballCtrl(radius=ctrlSize*.25, name='%s_ik_%s_ctrl' % (self.name, num))
            b = coreUtils.addParent(c, 'group', '%s_ik_buffer_%s_srt' % (self.name, num))
            b.t.set(cvs[i])
            b.setParent(self.interfaceGrp)
            d = coreUtils.decomposeMatrix(c.worldMatrix[0], name='%s_ikWorldMtxToSrt_%s_utl' % (self.name, num))
            d.outputTranslate.connect(ikSoftCrv.controlPoints[i])
            componentUtils.addSpaceSwitch(node=b, name='ik_%s' % num, spaces=['base'], type='parent', ctrlNode=c, targetNodes=[baseCtrl])
            self.ctrls.append(c)
            self.ikCtrls.append(c)

            # add attributes for hardening curves
            pmc.addAttr(c, ln='inner_sharpness', at='double', minValue=0, maxValue=1, k=1, h=0)
            pmc.addAttr(c, ln='outer_sharpness', at='double', minValue=0, maxValue=1, k=1, h=0)
            pmc.addAttr(c, ln='sharpness', at='double', minValue=0, maxValue=0.5, defaultValue=.33, k=1, h=0)

        # Connect endpoints of hard curve and soft curve
        coreUtils.connectAttrToMany(ikSoftCrv.controlPoints[0], [ikHardCrv.controlPoints[0], ikHardCrv.controlPoints[1]])
        coreUtils.connectAttrToMany(ikSoftCrv.controlPoints[numIkCtrls-1], [ikHardCrv.controlPoints[(numIkCtrls*3)-3], ikHardCrv.controlPoints[(numIkCtrls*3)-4]])

        # Connect interior controls points and set up blending for hardness
        for i in range(1, numIkCtrls-1):
            num = str(i+1).zfill(2)
            inBc = coreUtils.blend(ikSoftCrv.controlPoints[i-1], ikSoftCrv.controlPoints[i],
                                   '%s_ikInHardnessBlend_%s_utl' % (self.name, num),
                                   blendAttr=self.ikCtrls[i].sharpness)
            outBc = coreUtils.blend(ikSoftCrv.controlPoints[i+1], ikSoftCrv.controlPoints[i],
                                   '%s_ikOutHardnessBlend_%s_utl' % (self.name, num),
                                   blendAttr=self.ikCtrls[i].sharpness)

            inBc.output.connect(ikHardCrv.controlPoints[(i*3)-1])
            outBc.output.connect(ikHardCrv.controlPoints[(i*3)+1])
            ikSoftCrv.controlPoints[i].connect(ikHardCrv.controlPoints[i*3])

        # Arclen stuff for stretching vs maintaining length
        stretch = curveUtils.getCurveStretch(ikSoftCrv, name='%s' % self.name)

        blendCond = pmc.createNode('condition', name='%s_stretchIsNegative_utl' % self.name)
        stretch['stretchFactor'].outputX.connect(blendCond.firstTerm)
        self.mainGrp.globalScale.connect(stretch['globalScaleFactor'].input1X)
        blendCond.operation.set(4)
        blendCond.secondTerm.set(1.0)
        blendCond.colorIfFalseR.set(1.0)
        stretch['stretchFactor'].outputX.connect(blendCond.colorIfTrueR)

        stretchGoal = coreUtils.blend(1.0, baseCtrl.stretch, name='%s_stretchGoal_utl' % self.name, blendAttr=blendCond.outColorR)
        rangeMax = coreUtils.blend(stretchGoal.outputR, 1.0, name='%s_rangeMax_utl' % self.name, blendAttr=baseCtrl.anchor)
        rangeMin = coreUtils.minus([rangeMax.outputR, stretchGoal.outputR], name='%s_rangeMin_utl' % self.name)

        tailParam = coreUtils.blend(rangeMin.output1D, rangeMax.outputR,  name='%s_tailParam_utl' % self.name, blendAttr=baseCtrl.tail_extend)
        headParam = coreUtils.blend(rangeMin.output1D, rangeMax.outputR, name='%s_headParam_utl' % self.name, blendAttr=baseCtrl.head_extend)

        # motionPath nodes for upnode joints
        upNodeMps = []
        for i in range(numFkCtrls):
            num = str(i+1).zfill(2)
            mp = pmc.createNode('motionPath', name='%s_upNodeMp_%s_utl' % (self.name, num))
            mp.fractionMode.set(1)
            ikSoftCrv.worldSpace[0].connect(mp.geometryPath)
            param = (1.0 / (numFkCtrls-1)) * i
            paramBlend = coreUtils.blend(tailParam.outputR, headParam.outputR, name='%s_upNodeParam_%s_utl' % (self.name, num))
            paramBlend.blender.set(param)
            paramBlend.outputR.connect(mp.uValue)
            upNodeMps.append(mp)

        # IK joints to use for upnodes
        upVecs=[]
        upVecs.append(coreUtils.matrixAxisToVector(baseCtrl, name='%s_baseUpVector_utl' % self.name, axis='y'))
        for i in range(numFkCtrls):
            num = str(i+1).zfill(2)
            pmc.select(self.rigGrp)
            baseJnt = pmc.joint(name='%s_upVecBase_%s_jnt' % (self.name, num))
            endJnt = pmc.joint(name='%s_upVecEnd_%s_jnt' % (self.name, num))
            endJnt.tz.set(1.0)
            ikHandle = pmc.ikHandle(solver='ikRPsolver', name='%s_upVec_%s_ikHandle' % (self.name, num), startJoint=baseJnt, endEffector=endJnt, setupForRPsolver=1)[0]
            ikHandle.poleVector.set(0, 0, 0)
            upNodeMps[i].allCoordinates.connect(baseJnt.t)
            upNodeMps[i+1].allCoordinates.connect(ikHandle.t)
            upVecs.append(coreUtils.matrixAxisToVector(baseJnt, name='%s_ikUpVector_%s_utl' % (self.name, num), axis='y'))

        # Motion path nodes
        pathMatricesSoft = []
        pathMatricesSharp = []
        for i in range(numFkCtrls * jointsPerCtrl):
            num = str(i+1).zfill(2)
            mpSoft = pmc.createNode('motionPath', name='%s_mp_%s_soft_utl' % (self.name, num))
            mpSoft.fractionMode.set(1)
            mpSoft.follow.set(1)
            mpSoft.frontAxis.set(2)
            mpSoft.upAxis.set(1)
            mpSharp = pmc.createNode('motionPath', name='%s_mp_%s_hard_utl' % (self.name, num))
            mpSharp.fractionMode.set(1)
            mpSharp.follow.set(1)
            mpSharp.frontAxis.set(2)
            mpSharp.upAxis.set(1)
            poi = pmc.createNode('pointOnCurveInfo', name='%s_curveInfo_%s_utl' % (self.name, num))
            coreUtils.connectAttrToMany(ikSoftCrv.worldSpace[0], [mpSoft.geometryPath, mpSharp.geometryPath, poi.inputCurve])
            param = 1.0 / ((numFkCtrls * jointsPerCtrl)-1) * i
            paramBlend = coreUtils.blend(tailParam.outputR, headParam.outputR, name='%s_param_%s_utl' % (self.name, num))
            paramBlend.blender.set(param)
            coreUtils.connectAttrToMany(paramBlend.outputR, [mpSoft.uValue, mpSharp.uValue, poi.parameter])

            mtxSoft = pmc.createNode('composeMatri', name='%s_pathMtxSoft_%s_utl' % (self.name, num))
            mpSoft.allCoordinates.connect(mtxSoft.inputTranslate)
            mpSoft.rotate.connect(mtxSoft.inputRotate)
            pathMatricesSoft.append(mtxSoft)

            mtxSharp = pmc.createNode('composeMatri', name='%s_pathMtxSharp_%s_utl' % (self.name, num))
            mpSoft.allCoordinates.connect(mtxSharp.inputTranslate)
            mpSoft.rotate.connect(mtxSharp.inputRotate)
            pathMatricesSharp.append(mtxSharp)

            # Up vector stuff
            startUpVec = upVecs[0]
            endUpVec = upVecs[1]
            if i != 0:
                startUpVec = upVecs[numFkCtrls / i]
                endUpVec = upVecs[(numFkCtrls / i) + 1]

            upVecBlend = coreUtils.blend(startUpVec.output, endUpVec.output, name='%s_upVecBlend_%s_utl' % (self.name, num))
            upVecBlend.blender.set((1.0 / (numIkCtrls - 1) * (i%numIkCtrls)))
            coreUtils.connectAttrToMany(upVecBlend.output, [mpSoft.worldUpVector, mpSharp.worldUpVector])

        # Create Lattice
        pmc.select([])
        lattice = pmc.lattice(dv=(2, 2, numIkCtrls*latticeDivisions), scale=(2, 2, coreUtils.getDistance(start, end)), ro=(0, 0, 45))
        lattice[0].rename('%s_lattice_def' % self.name)
        lattice[1].rename('%s_lattice_cage' % self.name)
        lattice[2].rename('%s_lattice_base' % self.name)
        coreUtils.align(lattice[2], lattice[1], scale=1)

        # create vectors and blends for each lattice point
        for i in range(numFkCtrls * jointsPerCtrl):
            num = str(i+1).zfill(2)
            xPosSoftVp = coreUtils.pointMatrixMult((1,0,0), pathMatricesSoft[i].outputMatrix, name='%s_xPosSoft_%s_utl' % (self.name, num))
            xPosSharpVp = coreUtils.pointMatrixMult((1,0,0), pathMatricesSharp[i].outputMatrix, name='%s_xPosSharp_%s_utl' % (self.name, num))
            xPosBc = coreUtils.blend(xPosSoftVp.output, xPosSharpVp.output, name='%s_sPosBlend_%s_utl' % (self.name, num))








    def cleanUp(self):
        super(DrSnake, self).cleanUp()
