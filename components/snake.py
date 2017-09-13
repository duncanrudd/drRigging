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
        radius=1

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

        ikHardCrv = curveUtils.curveBetweenNodes(start, end, numCVs=(numIkCtrls*3)+3, name='%s_ik_hard_crv' % self.name)
        ikHardCrv.setParent(self.rigGrp)

        pmc.rebuildCurve(ikSoftCrv, ch=0, rpo=1, kr=0, s=numIkCtrls-1, d=3)

        # ik controls
        cvs = ikSoftCrv.getCVs(space='world')
        self.ikCtrls = []
        for i in range(numIkCtrls+2):
            num = str(i+1).zfill(2)
            c = controls.ballCtrl(radius=ctrlSize*.25, name='%s_ik_%s_ctrl' % (self.name, num))
            b = coreUtils.addParent(c, 'group', '%s_ik_buffer_%s_srt' % (self.name, num))
            b.t.set(cvs[i])
            b.setParent(self.interfaceGrp)
            if i != 1 and i != (numIkCtrls):
                componentUtils.addSpaceSwitch(node=b, name='ik_%s' % num, spaces=['base'], type='parent', ctrlNode=c, targetNodes=[baseCtrl])
            d = coreUtils.decomposeMatrix(c.worldMatrix[0], name='%s_ikWorldMtxToSrt_%s_utl' % (self.name, num))
            d.outputTranslate.connect(ikSoftCrv.controlPoints[i])
            self.ctrls.append(c)
            self.ikCtrls.append(c)

            # add attributes for hardening curves
            pmc.addAttr(c, ln='inner_sharpness', at='double', minValue=0, maxValue=1, k=1, h=0)
            pmc.addAttr(c, ln='outer_sharpness', at='double', minValue=0, maxValue=1, k=1, h=0)
            pmc.addAttr(c, ln='sharpness', at='double', minValue=0, maxValue=0.5, defaultValue=.33, k=1, h=0)

        self.ikCtrls[1].getParent().setParent(self.ikCtrls[0])
        self.ikCtrls[-2].getParent().setParent(self.ikCtrls[-1])

        # Connect endpoints of hard curve and soft curve
        coreUtils.connectAttrToMany(ikSoftCrv.controlPoints[0], [ikHardCrv.controlPoints[0], ikHardCrv.controlPoints[1]])
        coreUtils.connectAttrToMany(ikSoftCrv.controlPoints[len(cvs)-1], [ikHardCrv.controlPoints[len(ikHardCrv.getCVs())-1], ikHardCrv.controlPoints[len(ikHardCrv.getCVs())-2]])

        # Connect interior controls points and set up blending for hardness
        for i in range(1, numIkCtrls+1):
            num = str(i+1).zfill(2)
            inBc = coreUtils.blend(ikSoftCrv.controlPoints[i-1], ikSoftCrv.controlPoints[i],
                                   '%s_ikInHardnessBlend_%s_utl' % (self.name, num),
                                   blendAttr=self.ikCtrls[i].sharpness)
            outBc = coreUtils.blend(ikSoftCrv.controlPoints[i+1], ikSoftCrv.controlPoints[i],
                                   '%s_ikOutHardnessBlend_%s_utl' % (self.name, num),
                                   blendAttr=self.ikCtrls[i].sharpness)

            inBc.output.connect(ikHardCrv.controlPoints[(i*3)-1])
            if i != numIkCtrls:
                outBc.output.connect(ikHardCrv.controlPoints[(i*3)+1])
            ikSoftCrv.controlPoints[i].connect(ikHardCrv.controlPoints[i*3])

        '''
        # Create Lattice
        pmc.select([])
        lattice = pmc.lattice(dv=(2, 2, numIkCtrls*latticeDivisions), scale=(2, 2, coreUtils.getDistance(start, end)), ro=(0, 0, 45))
        lattice[0].rename('%s_lattice_def' % self.name)
        lattice[1].rename('%s_lattice_cage' % self.name)
        lattice[2].rename('%s_lattice_base' % self.name)
        coreUtils.align(lattice[2], lattice[1], scale=1)
        '''
        # point on curve info nodes for jointa
        softSamples = curveUtils.sampleCurve(crv=ikSoftCrv, numSamples=(((numIkCtrls-1) * latticeDivisions)+1), name='%s_soft' % self.name, even=1)
        sharpSamples = curveUtils.sampleCurve(crv=ikHardCrv, numSamples=(((numIkCtrls-1) * latticeDivisions)+1), name='%s_sharp' % self.name, even=1)

        '''
        # IK joints
        for i in range(numIkCtrls * latticeDivisions):
            num = str(i+1).zfill(2)
            pmc.select(self.rigGrp)
            baseJnt = pmc.joint(name='%s_softBase_%s_jnt' % (self.name, num))
            endJnt = pmc.joint(name='%s_softEnd_%s_jnt' % (self.name, num))
            endJnt.tz.set(1.0)
            ikHandle = pmc.ikHandle(solver='ikRPsolver', name='%s_soft_%s_ikHandle' % (self.name, num), startJoint=baseJnt, endEffector=endJnt, setupForRPsolver=1)[0]
            ikHandle.poleVector.set(0, 0, 0)
            softSamples[i].result.position.connect(baseJnt.t)
            handlePos = coreUtils.add([softSamples[i].result.position, softSamples[i].result.normalizedTangent], name='%s_softTangentOffset_%s_utl' % (self.name, num))
            handlePos.output3D.connect(ikHandle.t)
        '''


        # Matrices along path using previous sample for upVec
        '''
        for i in range((numIkCtrls+2)*latticeDivisions):
            num=str(i+1).zfill(2)
            mtx = pmc.createNode('fourByFourMatrix', name='%s_pathMtx_%s_utl' % (self.name, num))
            upVec = None
            if i == 0:
                upVec = coreUtils.matrixAxisToVector(self.ikCtrls[0], name='%s_upVecSoft_%s_utl' % (self.name, num), axis='y')
            else:
                upVec = pmc.createNode('vectorProduct', name='%s_upVecSoft_%s_utl' % (self.name, num))
                upVec.operation.set(0)
                upVec.normalizeOutput.set(1)
                matricesSoft[-1].in10.connect(upVec.input1X)
                matricesSoft[-1].in11.connect(upVec.input1Y)
                matricesSoft[-1].in12.connect(upVec.input1Z)
            xVp = coreUtils.cross(softSamples[i].result.normalizedTangent, upVec.output, name='%s_xVecSoft_%s_utl' % (self.name, num))
            yVp = coreUtils.cross(xVp.output, softSamples[i].result.normalizedTangent, name='%s_yVecSoft_%s_utl' % (self.name, num))

            xVp.outputX.connect(mtx.in00)
            xVp.outputY.connect(mtx.in01)
            xVp.outputZ.connect(mtx.in02)

            yVp.outputX.connect(mtx.in10)
            yVp.outputY.connect(mtx.in11)
            yVp.outputZ.connect(mtx.in12)

            softSamples[i].result.normalizedTangent.normalizedTangentX.connect(mtx.in20)
            softSamples[i].result.normalizedTangent.normalizedTangentY.connect(mtx.in21)
            softSamples[i].result.normalizedTangent.normalizedTangentZ.connect(mtx.in22)

            softSamples[i].result.position.positionX.connect(mtx.in30)
            softSamples[i].result.position.positionY.connect(mtx.in31)
            softSamples[i].result.position.positionZ.connect(mtx.in32)

            d = coreUtils.decomposeMatrix(mtx.output, name='%s_mtxToSrtSoft_%s_utl' % (self.name, num))

            matricesSoft.append(mtx)
            srtsSoft.append(d)

            g = pmc.group(empty=1)
            d.outputTranslate.connect(g.t)
            d.outputRotate.connect(g.r)
        '''
        upVecs = []
        ikMainCtrls = [self.ikCtrls[i] for i in range(len(self.ikCtrls)) if not i == 1 and not i == (len(self.ikCtrls)-2)]
        srtsSoft = []
        srtsSharp = []

        def _makePathMtx(name, samp, bc):
                mtx = pmc.createNode('fourByFourMatrix', name='%s_pathMtx%s_utl' % (self.name, name))

                yVp = coreUtils.cross(samp.result.normalizedTangent, bc, name='%s_yVec%s_utl' % (self.name, name))
                xVp = coreUtils.cross(yVp.output, samp.result.normalizedTangent, name='%s_xVec%s_utl' % (self.name, name))

                xVp.outputX.connect(mtx.in00)
                xVp.outputY.connect(mtx.in01)
                xVp.outputZ.connect(mtx.in02)

                yVp.outputX.connect(mtx.in10)
                yVp.outputY.connect(mtx.in11)
                yVp.outputZ.connect(mtx.in12)

                samp.result.normalizedTangent.normalizedTangentX.connect(mtx.in20)
                samp.result.normalizedTangent.normalizedTangentY.connect(mtx.in21)
                samp.result.normalizedTangent.normalizedTangentZ.connect(mtx.in22)

                samp.result.position.positionX.connect(mtx.in30)
                samp.result.position.positionY.connect(mtx.in31)
                samp.result.position.positionZ.connect(mtx.in32)

                d = coreUtils.decomposeMatrix(mtx.output, name='%s_mtxToSrt%s_utl' % (self.name, name))
                return d

        for i in range(len(self.ikCtrls)-3):
            segStart = ikMainCtrls[i]
            segEnd = ikMainCtrls[i+1]
            if i == 0:
                upVecs.append(coreUtils.matrixAxisToVector(segStart, name='%s_upVec_%s_utl' % (self.name, str(i+1).zfill(2)), axis='x'))
            startVp = upVecs[-1]
            endVp = coreUtils.matrixAxisToVector(segEnd, name='%s_upVec_%s_utl' % (self.name, str(i+1).zfill(2)), axis='x')
            for a in range(latticeDivisions):
                num = str((i+1)*(a+1)).zfill(2)
                samp = softSamples[(i*latticeDivisions) + a]
                bc = coreUtils.blend(endVp.output, startVp.output, name='%s_upVecBlend_%s_utl' % (self.name, num))
                bc.blender.set((1.0 / (latticeDivisions))*a)

                # soft matrix
                d = _makePathMtx('Soft_%s' % num, samp, bc.output)
                srtsSoft.append(d)

                g = pmc.group(empty=1)
                d.outputTranslate.connect(g.t)
                d.outputRotate.connect(g.r)

                # Sharp matrix
                samp = sharpSamples[(i*latticeDivisions) + a]
                if samp == sharpSamples[0]:
                    srtsSharp.append(d)
                else:
                    d = _makePathMtx('Sharp_%s' % num, samp, bc.output)

                    srtsSharp.append(d)
                    g = pmc.group(empty=1)
                    d.outputTranslate.connect(g.t)
                    d.outputRotate.connect(g.r)

            upVecs.append(endVp)

        d = _makePathMtx('Soft_%s' % len(softSamples), softSamples[-1], upVecs[-1].output)
        print softSamples[-1]
        srtsSoft.append(d)
        srtsSharp.append(d)
        g = pmc.group(empty=1)
        print g
        d.outputTranslate.connect(g.t)
        d.outputRotate.connect(g.r)


        # create linear curves to drive the lattice
        posYCrv = curveUtils.curveBetweenNodes(start, end, numCVs=numIkCtrls*latticeDivisions, name='%s_posY_crv' % self.name, degree=1)

        negYCrv = curveUtils.curveBetweenNodes(start, end, numCVs=numIkCtrls*latticeDivisions, name='%s_negY_crv' % self.name, degree=1)

        posXCrv = curveUtils.curveBetweenNodes(start, end, numCVs=numIkCtrls*latticeDivisions, name='%s_posX_crv' % self.name, degree=1)

        negXCrv = curveUtils.curveBetweenNodes(start, end, numCVs=numIkCtrls*latticeDivisions, name='%s_negX_crv' % self.name, degree=1)

        # create point field along hard and soft curves
        softPoints = []
        sharpPoints = []
        blendPoints = []

        def _makePoint(name, vector, sampleArray):
                point = pmc.createNode('plusMinusAverage', name='%s_%s_utl' % (self.name, name))
                point.input3D[0].set(vector)
                sampleArray[i].result.position.connect(point.input3D[1])
                return point

        for i in range(len(softSamples)):
            num = str(i+1).zfill(2)

            vecs = [(1, 0, 0), (-1, 0, 0), (0, 1, 0), (0, -1, 0)]
            names = ['posX', 'negX', 'posY', 'negY']
            for a in range(4):
                softPoints.append(_makePoint('%sSoft_%s' % (names[a], num), vecs[a], softSamples))
                sharpPoints.append(_makePoint('%sSharp_%s' % (names[a], num), vecs[a], sharpSamples))

        # Calculate weights for soft / sharp blending at each ik ctrl
        for i in range(len(ikMainCtrls)):
            num = num = str(i+1).zfill(2)

            curvatureVec = coreUtils.minus([softSamples[i].result.curvatureCenter, softSamples[i].result.position], '%s_curvatureVec_%s_utl' % (self.name, num))
            curvatureCond = pmc.createNode('condition', name='%s_isCurved_%s_utl' % (self.name, num))
            softSamples[i].curvatureRadius.connect(curvatureCond.firstTerm)
            curvatureCond.colorIfTrue.set((0, 1, 0))
            curvatureVec.output3D.connect(curvatureCond.colorIfFalse)
            curvatureVecNorm = coreUtils.normalizeVector(curvatureCond.outColor, name='%s_curvatureVecNormalized_%s_utl' % (self.name, num))



        '''
        # point on curve info nodes for measuring curvature at joints
        infoNodes = curveUtils.sampleCurve(crv=ikHardCrv, numSamples=numIkCtrls, name='ikCrv')

        # create vectors from info nodes to lattice points and from info nodes to curvature centres
        latticeVecsY = []
        latticeVecsX = []
        for i in range(numIkCtrls):
            num = str(i+1).zfill(2)
            pmaY = coreUtils.minus([posYCrvSharp.controlPoints[i*latticeDivisions], infoNodes[i].result.position], name='%s_vecY_%s_utl' % (self.name, num))
            vpY = coreUtils.normalizeVector(pmaY.output3D, name='%s_vecYNormalized_%s_utl' % (self.name, num))

            pmaX = coreUtils.minus([posXCrvSharp.controlPoints[i*latticeDivisions], infoNodes[i].result.position], name='%s_vecX_%s_utl' % (self.name, num))
            vpX = coreUtils.normalizeVector(pmaX.output3D, name='%s_vecXNormalized_%s_utl' % (self.name, num))

            pmaC = coreUtils.minus([infoNodes[i].result.curvatureCenter, infoNodes[i].result.position], name='%s_vecX_%s_utl' % (self.name, num))
            curvatureCond = pmc.createNode('condition', name='%s_isCurved_%s_utl' % (self.name, num))
            infoNodes[i].curvatureRadius.connect(curvatureCond.firstTerm)
            curvatureCond.colorIfTrue.set((0, 1, 0))
            pmaC.output3D.connect(curvatureCond.colorIfFalse)
            vpC = coreUtils.normalizeVector(curvatureCond.outColor, name='%s_vecYNormalized_%s_utl' % (self.name, num))

            dotY = coreUtils.dot(vpC.output, vpY.output, name='%s_dotY_%s_utl' % (self.name, num))
            dotYRemap = pmc.createNode('remapValue', name='%s_dotYNormalized_%s_utl' % (self.name, num))
            dotY.outputX.connect(dotYRemap.inputValue)
            dotYRemap.inputMin.set(-1)
            dotYRev = coreUtils.reverse(dotYRemap.outValue, name='%s_dotYReversed_%s_utl' % (self.name, num))

            dotX = coreUtils.dot(vpC.output, vpX.output, name='%s_dotX_%s_utl' % (self.name, num))
            dotXRemap = pmc.createNode('remapValue', name='%s_dotXNormalized_%s_utl' % (self.name, num))
            dotX.outputX.connect(dotXRemap.inputValue)
            dotXRemap.inputMin.set(-1)
            dotXRev = coreUtils.reverse(dotYRemap.outValue, name='%s_dotXReversed_%s_utl' % (self.name, num))

            innerMultY = coreUtils.multiply(dotYRemap.outValue, self.ikCtrls[i].inner_sharpness, name='%s_innerSharpenY_%s_utl' % (self.name, num))
            outerMultY = coreUtils.multiply(dotYRev.outputX, self.ikCtrls[i].outer_sharpness, name='%s_outerSharpenY_%s_utl' % (self.name, num))
            sharpenPosY = coreUtils.add([innerMultY.outputX, outerMultY.outputX], name='%s_sharpenPosY_%s_utl' % (self.name, num))
            sharpenNegY = coreUtils.convert(sharpenPosY.output1D, -1.0, name='%s_sharpenNegY_%s_utl' % (self.name, num))

            innerMultX = coreUtils.multiply(dotXRemap.outValue, self.ikCtrls[i].inner_sharpness, name='%s_innerSharpenX_%s_utl' % (self.name, num))
            outerMultX = coreUtils.multiply(dotXRev.outputX, self.ikCtrls[i].outer_sharpness, name='%s_outerSharpenX_%s_utl' % (self.name, num))
            sharpenPosX = coreUtils.add([innerMultX.outputX, outerMultX.outputX], name='%s_sharpenPosX_%s_utl' % (self.name, num))
            sharpenNegX = coreUtils.convert(sharpenPosX.output1D, -1.0, name='%s_sharpenNegX_%s_utl' % (self.name, num))

            bcPosY = coreUtils.blend(posYCrvSoft.controlPoints[i], posYCrvSharp.controlPoints[i], name='%s_blendPosY_%s_utl' % (self.name, num), blendAttr=sharpenPosY.output1D)
            bcPosY.output.connect(posYCrv.controlPoints[i])

            bcNegY = coreUtils.blend(negYCrvSoft.controlPoints[i], negYCrvSharp.controlPoints[i], name='%s_blendNegY_%s_utl' % (self.name, num), blendAttr=sharpenNegY.output)
            bcNegY.output.connect(negYCrv.controlPoints[i])

            bcPosX = coreUtils.blend(posXCrvSoft.controlPoints[i], posXCrvSharp.controlPoints[i], name='%s_blendPosX_%s_utl' % (self.name, num), blendAttr=sharpenPosX.output1D)
            bcPosX.output.connect(posXCrv.controlPoints[i])

            bcNegX = coreUtils.blend(negXCrvSoft.controlPoints[i], negXCrvSharp.controlPoints[i], name='%s_blendNegX_%s_utl' % (self.name, num), blendAttr=sharpenNegX.output)
            bcNegX.output.connect(negXCrv.controlPoints[i])
        '''




    def cleanUp(self):
        pass
        #super(DrSnake, self).cleanUp()
