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

import math

class DrSnake(drBase.DrBaseComponent):
    '''
    '''
    def __init__(self, start, end, name, numFkCtrls, jointsPerCtrl, numIkCtrls, latticeDivisions=4, axis='x', upAxis='y',  cleanUp=1):
        super(DrSnake, self).__init__(name)

        self.build(start, end, numFkCtrls, jointsPerCtrl, numIkCtrls, latticeDivisions, axis, upAxis)

        if cleanUp:
            # self.cleanUp()
            pass

    def build(self, start, end, numFkCtrls, jointsPerCtrl, numIkCtrls, latticeDivisions, axis, upAxis):
        start, end = coreUtils.getStartAndEnd(start, end)
        ctrlSize = coreUtils.getDistance(start, end) * .1
        radius=1

        # main control
        baseCtrl = controls.circleBumpCtrl(radius=ctrlSize,
                                           name='%s_base_ctrl' % self.name, axis=axis)
        baseSrt = coreUtils.decomposeMatrix(baseCtrl.worldMatrix[0], name='%s_baseWorldMtxToSrt_utl' % baseCtrl)
        baseBuffer = coreUtils.addParent(baseCtrl, 'group', name='%s_base_buffer_srt' % self.name)
        baseBuffer.setParent(self.interfaceGrp)
        self.ctrls.append(baseCtrl)

        points = coreUtils.pointsAlongVector(start, end, divisions=numIkCtrls-1)
        points.insert(1, coreUtils.pointsAlongVector(points[0], points[1], 2)[1])
        points.insert(numIkCtrls, coreUtils.pointsAlongVector(points[-2], points[-1], 2)[1])

        # ik ctrls
        ikCtrls=[]
        refVecs = []
        ikSrtList = []
        for i in range(len(points)):
            num = str(i+1).zfill(2)

            c = controls.ballCtrl(radius=ctrlSize*.5, name='%s_ik_%s_ctrl' % (self.name, num))
            b = coreUtils.addParent(c, 'group', '%s_ik_%s_buffer_srt' % (self.name, num))
            b.setParent(self.interfaceGrp)
            b.t.set(points[i])
            ikCtrls.append(c)
            refVecs.append(coreUtils.matrixAxisToVector(c, name='%s_referenceVec_%s_utl' % (self.name, num), axis='x'))
            ikSrtList.append(coreUtils.decomposeMatrix(c.worldMatrix[0], name='%s_ikCtrlMtxToSrt_%s_utl' % (self.name, num)))
        ikCtrls[1].getParent().setParent(ikCtrls[0])
        ikCtrls[-2].getParent().setParent(ikCtrls[-1])

        # tangent vectors
        tangentVecs = []
        tangentVecs.append(coreUtils.matrixAxisToVector(ikCtrls[0], name='%s_tangentVec_00_utl' % self.name, axis='z'))
        for i in range(len(ikCtrls)-1):
            num = str(i+1).zfill(2)
            vec = coreUtils.vectorBetweenNodes(ikCtrls[i], ikCtrls[i+1], name='%s_tangentVec_%s_utl' % (self.name, num))
            vecNorm = coreUtils.normalizeVector(vec.output3D, name='%s_tangentVecNorm_%s_utl' % (self.name, num))
            tangentVecs.append(vecNorm)

        tangentVecs.append(coreUtils.matrixAxisToVector(ikCtrls[-1], name='%s_tangentVec_%s_utl' % (self.name, str(len(points)).zfill(2)), axis='z'))

        blendedTangentVecs=[]
        # create blended tangent vector
        for i in range(len(ikCtrls)):
            num = str(i+1).zfill(2)
            bc = coreUtils.blend(tangentVecs[i].output, tangentVecs[i+1].output, name='%s_blendedTangentVec_%s_utl' % (self.name, num))
            bc.blender.set(0.5)
            blendedTangentVecNorm = coreUtils.normalizeVector(bc.output, name='%s_blendedTangentVecNorm_%s_utl' % (self.name, num))
            blendedTangentVecs.append(blendedTangentVecNorm)
        blendedTangentVecs.append(tangentVecs[-1])


        # create blend locator at each joint
        segMtxList = []
        segPoleVecList = []
        for i in range(len(ikCtrls)-1):
            num = str(i+1).zfill(2)
            segMtx = pmc.createNode('fourByFourMatrix', name='%s_segMtx_%s_utl' % (self.name, num))

            upVec = coreUtils.cross(tangentVecs[i+1].output, refVecs[i].output, name='%s_upVec_%s_utl' % (self.name, num))
            sideVec = coreUtils.cross(upVec.output, tangentVecs[i+1].output, name='%s_sideVec_%s_utl' % (self.name, num))

            sideVec.outputX.connect(segMtx.in00)
            sideVec.outputY.connect(segMtx.in01)
            sideVec.outputZ.connect(segMtx.in02)

            upVec.outputX.connect(segMtx.in10)
            upVec.outputY.connect(segMtx.in11)
            upVec.outputZ.connect(segMtx.in12)

            tangentVecs[i+1].outputX.connect(segMtx.in20)
            tangentVecs[i+1].outputY.connect(segMtx.in21)
            tangentVecs[i+1].outputZ.connect(segMtx.in22)

            d = coreUtils.isDecomposed(ikCtrls[i])
            d.outputTranslateX.connect(segMtx.in30)
            d.outputTranslateY.connect(segMtx.in31)
            d.outputTranslateZ.connect(segMtx.in32)

            segMtxList.append(segMtx)

            if i != 0:
                segMidPoint = coreUtils.blend(coreUtils.isDecomposed(ikCtrls[i-1]).outputTranslate, coreUtils.isDecomposed(ikCtrls[i+1]).outputTranslate, name='%s_segMidPoint_%s_utl' % (self.name, num))
                segMidPoint.blender.set(0.5)
                segPoleVec = coreUtils.minus([segMidPoint.output, coreUtils.isDecomposed(ikCtrls[i]).outputTranslate], name='%s_segPoleVec_%s_utl' % (self.name, num))
                segPoleVecCond = pmc.createNode('condition', name='%s_segPoleVecIsZero_%s_utl' % (self.name, num))
                segDot = coreUtils.dot(blendedTangentVecs[i].output, tangentVecs[i].output, name='%s_segDot_%s_utl' % (self.name, num))
                segDot.outputX.connect(segPoleVecCond.firstTerm)
                segPoleVecCond.secondTerm.set(1.0)
                refVecs[i].output.connect(segPoleVecCond.colorIfTrue)
                segPoleVec.output3D.connect(segPoleVecCond.colorIfFalse)
                segPoleVecNorm = coreUtils.normalizeVector(segPoleVecCond.outColor, name='%s_segPoleVecNorm_%s_utl' % (self.name, num))
                segPoleVecList.append(segPoleVecNorm)
            else:
                segPoleVecList.append(upVec)

        # create sharp and soft matrices at each lattice division
        posYList = []
        negYList = []
        posXList = []
        negXList = []

        sharpPointPositions = []

        for i in range(len(ikCtrls)-1):
            num = str(i+1).zfill(2)
            divs = latticeDivisions*2
            exceptionList = [0, 1, len(ikCtrls)-2, len(ikCtrls)-3]
            if i in exceptionList:
                divs = latticeDivisions
            isStraightCond = None
            if i < (len(ikCtrls)-2):
                isStraightCond = pmc.createNode('condition', name='%s_isStraight_%s_utl' % (self.name, num))
                tangentDot = coreUtils.dot(tangentVecs[i].output, blendedTangentVecs[i].output, name='%s_isStraightTangentDot_%s_utl' % (self.name, num))
                tangentDot.outputX.connect(isStraightCond.firstTerm)
                isStraightCond.secondTerm.set(1.0)
                segPoleVecList[i+1].output.connect(isStraightCond.colorIfTrue)
                segPoleVecList[i].output.connect(isStraightCond.colorIfFalse)

            for a in range(divs):
                num = str((i*divs)+a+1).zfill(2)
                blendVal = (1.0 / (divs))*a

                refVecBlend = coreUtils.blend(refVecs[i+1].output, refVecs[i].output, name='%s_refVecBlend_%s_utl' % (self.name, num))
                refVecBlend.blender.set(blendVal)
                refVecNorm = coreUtils.normalizeVector(refVecBlend.output, name='%s_pointRefVecNorm_%s_utl' % (self.name, num))

                pointPosBlend = coreUtils.blend(ikSrtList[i+1].outputTranslate, ikSrtList[i].outputTranslate, name='%s_pointPosBlend_%s_utl' % (self.name, num))
                pointPosBlend.blender.set(blendVal)
                sharpPointPositions.append(pointPosBlend)

                pointFrontVec = coreUtils.blend(blendedTangentVecs[i+1].output, blendedTangentVecs[i].output, name='%s_pointFrontVec_%s_utl' % (self.name, num))
                pointFrontVec.blender.set(blendVal)
                pointFrontVecNorm = coreUtils.normalizeVector(pointFrontVec.output, name='%s_pointFrontVecNorm_%s_utl' % (self.name, num))

                pointUpVec = coreUtils.cross(pointFrontVecNorm.output, refVecNorm.output, name='%s_pointUpVec_%s_utl' % (self.name, num))
                pointSideVec = coreUtils.cross(pointUpVec.output, pointFrontVecNorm.output, name='%s_pointUpVec_%s_utl' % (self.name, num))

                # Calculate X and Y scaling.
                # Get dot product between front vector and tangent vector
                scaleDot = coreUtils.dot(pointFrontVec.output, tangentVecs[i+1].output, name='%s_scaleDot_%s_utl' % (self.name, num))
                scaleMult = coreUtils.divide(1.0, scaleDot.outputX, name='%s_pointScaleMult_%s_utl' % (self.name, num))


                pointXVec = None
                if isStraightCond:
                    pointXVec = coreUtils.cross(pointFrontVecNorm.output, isStraightCond.outColor, name='%s_pointPoleVec_%s_utl' % (self.name, num))
                else:
                    pointXVec = coreUtils.cross(pointFrontVecNorm.output, segPoleVecList[i].output, name='%s_pointPoleVec_%s_utl' % (self.name, num))
                pointAngle = pmc.createNode('angleBetween', name='%s_pointAngle_%s_utl' % (self.name, num))
                pointRemap = pmc.createNode('remapValue', name='%s_pointAngleRemap_%s_utl' % (self.name, num))
                pointSideVec.output.connect(pointAngle.vector1)
                pointXVec.output.connect(pointAngle.vector2)
                uc = coreUtils.convert(pointAngle.angle, 57.296, name='%s_pointAngleRadToDeg_%s_utl' % (self.name, num))
                uc.output.connect(pointRemap.inputValue)
                pointRemap.inputMax.set(180.0)
                pointRemap.outputMin.set(-1.0)
                pointAngleAbs = coreUtils.forceAbsolute(pointRemap.outValue, name='%s_pointAngleAbs_%s_utl' % (self.name, num))

                scaleXBlend = coreUtils.blend(1.0, scaleMult.outputX, name='%s_pointXScale_%s_utl' % (self.name, num), blendAttr=pointAngleAbs[1].outputX)
                scaleYBlend = coreUtils.blend(scaleMult.outputX, 1.0, name='%s_pointYScale_%s_utl' % (self.name, num), blendAttr=pointAngleAbs[1].outputX)

                pointSideVecScaled = pmc.createNode('multiplyDivide', name='%s_pointSideVecScaled_%s_utl' % (self.name, num))
                coreUtils. connectAttrToMany(scaleXBlend.outputR, [pointSideVecScaled.input1X, pointSideVecScaled.input1Y, pointSideVecScaled.input1Z])
                pointSideVec.output.connect(pointSideVecScaled.input2)

                pointUpVecScaled = pmc.createNode('multiplyDivide', name='%s_pointUpVecScaled_%s_utl' % (self.name, num))
                coreUtils.connectAttrToMany(scaleYBlend.outputR, [pointUpVecScaled.input1X, pointUpVecScaled.input1Y, pointUpVecScaled.input1Z])
                pointUpVec.output.connect(pointUpVecScaled.input2)

                # Sharp matrix
                pointMtxSharp = pmc.createNode('fourByFourMatrix', name='%s_pointMtxSharp_%s_utl' % (self.name, num))

                pointSideVecScaled.outputX.connect(pointMtxSharp.in00)
                pointSideVecScaled.outputY.connect(pointMtxSharp.in01)
                pointSideVecScaled.outputZ.connect(pointMtxSharp.in02)

                pointUpVecScaled.outputX.connect(pointMtxSharp.in10)
                pointUpVecScaled.outputY.connect(pointMtxSharp.in11)
                pointUpVecScaled.outputZ.connect(pointMtxSharp.in12)

                pointFrontVecNorm.outputX.connect(pointMtxSharp.in20)
                pointFrontVecNorm.outputY.connect(pointMtxSharp.in21)
                pointFrontVecNorm.outputZ.connect(pointMtxSharp.in22)

                pointPosBlend.outputR.connect(pointMtxSharp.in30)
                pointPosBlend.outputG.connect(pointMtxSharp.in31)
                pointPosBlend.outputB.connect(pointMtxSharp.in32)

                posYList.append(coreUtils.pointMatrixMult((0, 1, 0), pointMtxSharp.output, name='%s_pointPosY_%s_utl' % (self.name, num)))
                negYList.append(coreUtils.pointMatrixMult((0, -1, 0), pointMtxSharp.output, name='%s_pointNegY_%s_utl' % (self.name, num)))
                posXList.append(coreUtils.pointMatrixMult((1, 0, 0), pointMtxSharp.output, name='%s_pointPosX_%s_utl' % (self.name, num)))
                negXList.append(coreUtils.pointMatrixMult((-1, 0, 0), pointMtxSharp.output, name='%s_pointNegX_%s_utl' % (self.name, num)))

                # Soft matrix
                '''
                pointMtxSoft = pmc.createNode('fourByFourMatrix', name='%s_pointMtxSoft_%s_utl' % (self.name, num))
                crvInf = pmc.createNode('pointOnCurveInfo', name='%s_softCrvInfo_%s_utl' % (self.name, num))
                softCrv.worldSpace[0].connect(crvInf.inputCurve)
                seg=i
                if i == len(ikCtrls)-2:
                    seg = i-2
                elif i > 0:
                    seg = i-1
                crvInf.parameter.set(seg + blendVal)

                softPointUpVec = coreUtils.cross(crvInf.normalizedTangent, pointSideVec.output, name='%s_softPointUpVec_%s_utl' % (self.name, num))

                pointSideVec.outputX.connect(pointMtxSoft.in00)
                pointSideVec.outputY.connect(pointMtxSoft.in01)
                pointSideVec.outputZ.connect(pointMtxSoft.in02)

                softPointUpVec.outputX.connect(pointMtxSoft.in10)
                softPointUpVec.outputY.connect(pointMtxSoft.in11)
                softPointUpVec.outputZ.connect(pointMtxSoft.in12)

                crvInf.normalizedTangentX.connect(pointMtxSoft.in20)
                crvInf.normalizedTangentY.connect(pointMtxSoft.in21)
                crvInf.normalizedTangentZ.connect(pointMtxSoft.in22)

                crvInf.positionX.connect(pointMtxSoft.in30)
                crvInf.positionY.connect(pointMtxSoft.in31)
                crvInf.positionZ.connect(pointMtxSoft.in32)

                '''


        posYList.append(coreUtils.pointMatrixMult((0, 1, 0), ikCtrls[-1].worldMatrix[0], name='%s_pointPosY_%s_utl' % (self.name, num)))
        negYList.append(coreUtils.pointMatrixMult((0, -1, 0), ikCtrls[-1].worldMatrix[0], name='%s_pointNegY_%s_utl' % (self.name, num)))
        posXList.append(coreUtils.pointMatrixMult((1, 0, 0), ikCtrls[-1].worldMatrix[0], name='%s_pointPosX_%s_utl' % (self.name, num)))
        negXList.append(coreUtils.pointMatrixMult((-1, 0, 0), ikCtrls[-1].worldMatrix[0], name='%s_pointNegX_%s_utl' % (self.name, num)))

        sharpPointPositions.append(coreUtils.isDecomposed(ikCtrls[-1]))

        # Soft curves
        softCrvs = []

        drivers = None
        for i in range(numIkCtrls):
            num = str(i+1).zfill(2)
            if i == 0:
                drivers = [sharpPointPositions[latticeDivisions/2].output,
                           sharpPointPositions[latticeDivisions].output,
                           sharpPointPositions[latticeDivisions+(latticeDivisions/2)].output]
            elif i == 1:
                drivers = [sharpPointPositions[latticeDivisions+(latticeDivisions/2)].output,
                           sharpPointPositions[latticeDivisions*2].output,
                           sharpPointPositions[latticeDivisions*3].output]
            elif i == numIkCtrls-2:
                drivers = [sharpPointPositions[latticeDivisions*((i*2)-1)].output,
                           sharpPointPositions[latticeDivisions*((i*2))].output,
                           sharpPointPositions[latticeDivisions*((i*2))+(latticeDivisions / 2)].output]
            elif i == numIkCtrls-1:
                drivers = [sharpPointPositions[latticeDivisions*((i*2)-1)-(latticeDivisions / 2)].output,
                           sharpPointPositions[latticeDivisions*((i*2)-1)].output,
                           sharpPointPositions[latticeDivisions*(i*2)-(latticeDivisions / 2)].output]
            else:
                drivers = [sharpPointPositions[latticeDivisions*((i*2)-1)].output,
                           sharpPointPositions[latticeDivisions*((i*2))].output,
                           sharpPointPositions[latticeDivisions*((i*2)+1)].output]
            crv = curveUtils.drivenCurve(drivers, name='%s_soft_%s_crv' % (self.name, num), degree=2, rebuild=0)
            crv.setParent(self.rigGrp)
            softCrvs.append(crv)

        softPointPositions = sharpPointPositions[:(latticeDivisions/2)]

        for i in range(numIkCtrls):
            numPoints = latticeDivisions*2
            if i == 0 or i == numIkCtrls-1:
                numPoints = latticeDivisions
            elif i == 1 or i == numIkCtrls-2:
                numPoints = int(latticeDivisions*1.5)

            for a in range(numPoints):
                num = str(len(softPointPositions)+1).zfill(2)
                blendVal = (1.0/numPoints)*a
                crvInf = pmc.createNode('pointOnCurveInfo', name='%s_softCrvInfo_%s_utl' % (self.name, num))
                softCrvs[i].worldSpace[0].connect(crvInf.inputCurve)
                crvInf.parameter.set(blendVal)
                softPointPositions.append(crvInf)

        for i in range((latticeDivisions/2)+1):
            index = len(softPointPositions)
            softPointPositions.append(sharpPointPositions[index])

        for i in range(len(softPointPositions)-1):
            loc = pmc.spaceLocator()
            try:
                softPointPositions[i].output.connect(loc.t)
            except:
                softPointPositions[i].position.connect(loc.t)
        loc = pmc.spaceLocator()
        softPointPositions[-1].outputTranslate.connect(loc.t)











        # create lattice
        latticeScale = math.sqrt(1*2)
        pmc.select([])
        lattice = pmc.lattice(dv=(2, 2, len(posYList)), scale=(1, 1, 1))
        lattice[0].rename('%s_lattice_def' % self.name)
        lattice[1].rename('%s_lattice_cage' % self.name)
        lattice[2].rename('%s_lattice_base' % self.name)
        lattice[2].rz.set(45)
        lattice[2].s.set((latticeScale, latticeScale, coreUtils.getDistance(start, end)))

        for i in range(len(posYList)):
            posYList[i].output.connect(lattice[1].controlPoints[(i*4)+3])
            negYList[i].output.connect(lattice[1].controlPoints[i*4])
            posXList[i].output.connect(lattice[1].controlPoints[(i*4)+1])
            negXList[i].output.connect(lattice[1].controlPoints[(i*4)+2])

        '''
        SOFT POINTS
        -- Based on a degree 2 interpolation of sharp points.
        -- Soft segments run from mid point to midpoint of sharp segments apart from at the ends where they run from the end to the following mid point.
        -- Points at midpoints do not change position based on soft vs sharp
        -- Each point is only affected by the softness parameters of a single joint
        '''







