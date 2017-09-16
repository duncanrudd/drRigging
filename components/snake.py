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

        upVecs = []
        ikMainCtrls = [self.ikCtrls[i] for i in range(len(self.ikCtrls)) if not i == 1 and not i == (len(self.ikCtrls)-2)]
        srtsSoft = []
        srtsSharp = []

        mtxListSoft = []
        mtxListSharp = []

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

                return mtx

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
                mtxListSoft.append(_makePathMtx('Soft_%s' % num, samp, bc.output))
                d = coreUtils.decomposeMatrix(mtxListSoft[-1].output, name='%s_mtxToSrtSoft_%s_utl' % (self.name, num))
                srtsSoft.append(d)

                g = pmc.group(empty=1)
                d.outputTranslate.connect(g.t)
                d.outputRotate.connect(g.r)

                # Sharp matrix
                samp = sharpSamples[(i*latticeDivisions) + a]
                if samp == sharpSamples[0]:
                    mtxListSharp.append(mtxListSoft[-1])
                    srtsSharp.append(d)
                else:
                    mtxListSharp.append(_makePathMtx('Sharp_%s' % num, samp, bc.output))
                    d = coreUtils.decomposeMatrix(mtxListSharp[-1].output, name='%s_mtxToSrtSharp_%s_utl' % (self.name, num))

                    srtsSharp.append(d)
                    g = pmc.group(empty=1)
                    d.outputTranslate.connect(g.t)
                    d.outputRotate.connect(g.r)

            upVecs.append(endVp)

        mtxListSoft.append(_makePathMtx('Soft_%s' % len(softSamples), softSamples[-1], upVecs[-1].output))
        mtxListSharp.append(mtxListSoft[-1])
        d = coreUtils.decomposeMatrix(mtxListSoft[-1].output, name='%s_mtxToSrtSharp_%s_utl' % (self.name, num))
        srtsSoft.append(d)
        srtsSharp.append(d)
        g = pmc.group(empty=1)
        d.outputTranslate.connect(g.t)
        d.outputRotate.connect(g.r)

        # create point field along hard and soft curves
        softPoints = []
        sharpPoints = []
        localPoints = []
        blendPoints = []

        def _makePoint(name, vector, matrix):
                point = coreUtils.pointMatrixMult(vector, matrix, name='%s_%s_utl' % (self.name, name))
                return point

        for i in range(len(softSamples)):
            num = str(i+1).zfill(2)

            vecs = [(1, 0, 0), (-1, 0, 0), (0, 1, 0), (0, -1, 0)]
            names = ['posX', 'negX', 'posY', 'negY']
            pointDictSoft = {}
            pointDictSharp = {}
            for a in range(4):
                pointDictSoft['%s' % names[a]] = _makePoint('%sSoft_%s' % (names[a], num), vecs[a], mtxListSoft[i].output)
                pointDictSharp['%s' % names[a]] = _makePoint('%sSharp_%s' % (names[a], num), vecs[a], mtxListSharp[i].output)
            softPoints.append(pointDictSoft)
            sharpPoints.append(pointDictSharp)
            localDict = {}
            localDict['y'] = coreUtils.minus([pointDictSharp['posY'].output, sharpSamples[i].result.position], name='%s_localPointY_%s_utl' % (self.name, num))
            localDict['x'] = coreUtils.minus([pointDictSharp['posX'].output, sharpSamples[i].result.position], name='%s_localPointX_%s_utl' % (self.name, num))
            localPoints.append(localDict)

        # Calculate weights for soft / sharp blending at each ik ctrl
        sharpenPosYList = []
        sharpenNegYList = []
        sharpenPosXList = []
        sharpenNegXList = []
        for i in range(len(ikMainCtrls)):
            num = str(i+1).zfill(2)

            # Get a normalized vector from the sample point on the sharp curve to its curvature centre
            samp = softSamples[(i*latticeDivisions)]
            curvatureVec = coreUtils.minus([samp.result.curvatureCenter, samp.result.position], '%s_curvatureVec_%s_utl' % (self.name, num))
            curvatureCond = pmc.createNode('condition', name='%s_isCurved_%s_utl' % (self.name, num))
            samp.curvatureRadius.connect(curvatureCond.firstTerm)
            curvatureCond.colorIfTrue.set((0, 1, 0))
            curvatureVec.output3D.connect(curvatureCond.colorIfFalse)
            curvatureVecNorm = coreUtils.normalizeVector(curvatureCond.outColor, name='%s_curvatureVecNormalized_%s_utl' % (self.name, num))

            # Get the dot product between the top and bottom points and the curvature vector
            dotY = coreUtils.dot(curvatureVecNorm.output, localPoints[i*latticeDivisions]['y'].output3D, name='%s_dotY_%s_utl' % (self.name, num))
            dotYRemap = pmc.createNode('remapValue', name='%s_dotYNormalized_%s_utl' % (self.name, num))
            dotY.outputX.connect(dotYRemap.inputValue)
            dotYRemap.inputMin.set(-1)
            dotYRev = coreUtils.reverse(dotYRemap.outValue, name='%s_dotYReversed_%s_utl' % (self.name, num))

            # Get the dot product between the left and right points and the curvature vector
            dotX = coreUtils.dot(curvatureVecNorm.output, localPoints[i*latticeDivisions]['x'].output3D, name='%s_dotX_%s_utl' % (self.name, num))
            dotXRemap = pmc.createNode('remapValue', name='%s_dotXNormalized_%s_utl' % (self.name, num))
            dotX.outputX.connect(dotXRemap.inputValue)
            dotXRemap.inputMin.set(-1)
            dotXRev = coreUtils.reverse(dotYRemap.outValue, name='%s_dotXReversed_%s_utl' % (self.name, num))

            # Multiply dot product results by inner and outer sharpness attribute values
            innerMultY = coreUtils.multiply(dotYRemap.outValue, ikMainCtrls[i].inner_sharpness, name='%s_innerSharpenY_%s_utl' % (self.name, num))
            outerMultY = coreUtils.multiply(dotYRev.outputX, ikMainCtrls[i].outer_sharpness, name='%s_outerSharpenY_%s_utl' % (self.name, num))
            sharpenPosYList.append(coreUtils.add([innerMultY.outputX, outerMultY.outputX], name='%s_sharpenPosY_%s_utl' % (self.name, num)))
            sharpenNegYList.append(coreUtils.reverse(sharpenPosYList[-1].output1D, name='%s_sharpenNegY_%s_utl' % (self.name, num)))

            innerMultX = coreUtils.multiply(dotXRemap.outValue, ikMainCtrls[i].inner_sharpness, name='%s_innerSharpenX_%s_utl' % (self.name, num))
            outerMultX = coreUtils.multiply(dotXRev.outputX, ikMainCtrls[i].outer_sharpness, name='%s_outerSharpenX_%s_utl' % (self.name, num))
            sharpenPosXList.append(coreUtils.add([innerMultX.outputX, outerMultX.outputX], name='%s_sharpenPosX_%s_utl' % (self.name, num)))
            sharpenNegXList.append(coreUtils.reverse(sharpenPosXList[-1].output1D, name='%s_sharpenNegX_%s_utl' % (self.name, num)))

        # For each sample, set up its blending between soft and hard curves
        pointsPosY = []
        pointsNegY = []
        pointsPosX = []
        pointsNegX = []

        def _makeBlendPoint(name, outerIndex, innerIndex, posList):
            index = (outerIndex*latticeDivisions)+innerIndex

            weightBc = pmc.createNode('blendTwoAttr', name='%s_%sWeightBlend_%s_utl' % (self.name, name, index))
            if 'pos' in name:
                posList[outerIndex].output1D.connect(weightBc.input[0])
                posList[outerIndex+1].output1D.connect(weightBc.input[1])
            else:
                posList[outerIndex].outputX.connect(weightBc.input[0])
                posList[outerIndex+1].outputX.connect(weightBc.input[1])
            weightBc.attributesBlender.set((1.0 / (latticeDivisions)*a))

            pointBc = coreUtils.blend(sharpPoints[index][name].output, softPoints[index][name].output, name='%s_%sPointBlend_%s_utl' % (self.name, name, index), blendAttr=weightBc.output)
            return pointBc

        for i in range(len(ikMainCtrls)-1):
            for a in range(latticeDivisions):
                pointsPosY.append(_makeBlendPoint('posY', i, a, sharpenPosYList))
                pointsNegY.append(_makeBlendPoint('negY', i, a, sharpenNegYList))
                pointsPosX.append(_makeBlendPoint('posX', i, a, sharpenPosXList))
                pointsNegX.append(_makeBlendPoint('negX', i, a, sharpenNegXList))

        num = str(len(pointsPosY)).zfill(2)
        pointsPosY.append(coreUtils.blend(sharpPoints[-1]['posY'].output, softPoints[-1]['posY'].output, name='%s_posYPointBlend_%s_utl' % (self.name, num)))
        pointsNegY.append(coreUtils.blend(sharpPoints[-1]['negY'].output, softPoints[-1]['negY'].output, name='%s_negYPointBlend_%s_utl' % (self.name, num)))
        pointsPosX.append(coreUtils.blend(sharpPoints[-1]['posX'].output, softPoints[-1]['posX'].output, name='%s_posXPointBlend_%s_utl' % (self.name, num)))
        pointsNegX.append(coreUtils.blend(sharpPoints[-1]['negX'].output, softPoints[-1]['negX'].output, name='%s_negXPointBlend_%s_utl' % (self.name, num)))


        # Make linear curves that will drive the lattice
        posYCrv = curveUtils.curveBetweenNodes(start, end, numCVs=len(pointsPosY), name='%s_posYWire_crv' % self.name, degree=1)
        negYCrv = curveUtils.curveBetweenNodes(start, end, numCVs=len(pointsPosY), name='%s_negYWire_crv' % self.name, degree=1)
        posXCrv = curveUtils.curveBetweenNodes(start, end, numCVs=len(pointsPosY), name='%s_posXWire_crv' % self.name, degree=1)
        negXCrv = curveUtils.curveBetweenNodes(start, end, numCVs=len(pointsPosY), name='%s_negXWire_crv' % self.name, degree=1)

        for i in range(len(pointsPosY)):
            pointsPosY[i].output.connect(posYCrv.controlPoints[i])
            pointsNegY[i].output.connect(negYCrv.controlPoints[i])
            pointsPosX[i].output.connect(posXCrv.controlPoints[i])
            pointsNegX[i].output.connect(negXCrv.controlPoints[i])

    def cleanUp(self):
        pass
        #super(DrSnake, self).cleanUp()
