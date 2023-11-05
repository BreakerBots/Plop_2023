
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.Optional;

import edu.wpi.first.math.geometry.Pose2d;

import static frc.robot.Constants.ScoreingConstants.*;

 /** Add your docs here. */
public enum Node {
    //-------------------- BLUE RIGHT GROUP --------------------
    C0_L(NodeCoulmn.C0, NodeHeight.LOW, NodeType.HYBRID),
    C0_M(NodeCoulmn.C0, NodeHeight.MID, NodeType.CONE),
    C0_H(NodeCoulmn.C0, NodeHeight.HIGH, NodeType.CONE),

    C1_L(NodeCoulmn.C1, NodeHeight.LOW, NodeType.HYBRID),
    C1_M(NodeCoulmn.C1, NodeHeight.MID, NodeType.CUBE),
    C1_H(NodeCoulmn.C1, NodeHeight.HIGH, NodeType.CUBE),

    C2_L(NodeCoulmn.C2, NodeHeight.LOW, NodeType.HYBRID),
    C2_M(NodeCoulmn.C2, NodeHeight.MID, NodeType.CONE),
    C2_H(NodeCoulmn.C2, NodeHeight.HIGH, NodeType.CONE),
    //-------------------- BLUE CENTER GROUP --------------------
    C3_L(NodeCoulmn.C3, NodeHeight.LOW, NodeType.HYBRID),
    C3_M(NodeCoulmn.C3, NodeHeight.MID, NodeType.CONE),
    C3_H(NodeCoulmn.C3, NodeHeight.HIGH, NodeType.CONE),

    C4_L(NodeCoulmn.C4, NodeHeight.LOW, NodeType.HYBRID),
    C4_M(NodeCoulmn.C4, NodeHeight.MID, NodeType.CUBE),
    C4_H(NodeCoulmn.C4, NodeHeight.HIGH, NodeType.CUBE),

    C5_L(NodeCoulmn.C5, NodeHeight.LOW, NodeType.HYBRID),
    C5_M(NodeCoulmn.C5, NodeHeight.MID, NodeType.CONE),
    C5_H(NodeCoulmn.C5, NodeHeight.HIGH, NodeType.CONE),
    //-------------------- BLUE LEFT GROUP --------------------
    C6_L(NodeCoulmn.C6, NodeHeight.LOW, NodeType.HYBRID),
    C6_M(NodeCoulmn.C6, NodeHeight.MID, NodeType.CONE),
    C6_H(NodeCoulmn.C6, NodeHeight.HIGH, NodeType.CONE),

    C7_L(NodeCoulmn.C7, NodeHeight.LOW, NodeType.HYBRID),
    C7_M(NodeCoulmn.C7, NodeHeight.MID, NodeType.CUBE),
    C7_H(NodeCoulmn.C7, NodeHeight.HIGH, NodeType.CUBE),

    C8_L(NodeCoulmn.C8, NodeHeight.LOW, NodeType.HYBRID),
    C8_M(NodeCoulmn.C8, NodeHeight.MID, NodeType.CONE),
    C8_H(NodeCoulmn.C8, NodeHeight.HIGH, NodeType.CONE);

    private NodeCoulmn coulmn;
    private NodeHeight height; 
    private NodeType type;
    private Node(NodeCoulmn coulmn, NodeHeight height, NodeType type) {
        this.coulmn = coulmn;
        this.height = height;
        this.type = type;
    }

    public NodeCoulmn getCoulmn() {
        return coulmn;
    }

    public NodeHeight getHeight() {
        return height;
    }

    public NodeType getType() {
        return type;
    }

    public Pose2d getExtensionAllignmentPose() {
        return new Pose2d(coulmn.getBlueBaseAllignmentPose().getX() + height.getExtensionOffset() + type.getExtensionOffset(), coulmn.getBlueBaseAllignmentPose().getY(), coulmn.getBlueBaseAllignmentPose().getRotation());
    }

    public Pose2d getAllignmentPose() {
        return coulmn.getBlueBaseAllignmentPose();
    }
 
    public static Optional<Node> fromCoulmnAndHeight(NodeCoulmn tgtCoulmn, NodeHeight tgtHeight) {
        for (Node node : values()) {
            if (node.getCoulmn() == tgtCoulmn && node.getHeight() == tgtHeight) {
                return Optional.of(node);
            }
        }
        return Optional.empty();
    }
    public enum NodeCoulmn {
        C0(C0_BASE_ALLIGNMENT_POSE), //Far right for blue driver, far left for red
        C1(C1_BASE_ALLIGNMENT_POSE),
        C2(C2_BASE_ALLIGNMENT_POSE),
        C3(C3_BASE_ALLIGNMENT_POSE),
        C4(C4_BASE_ALLIGNMENT_POSE),
        C5(C5_BASE_ALLIGNMENT_POSE),
        C6(C6_BASE_ALLIGNMENT_POSE),
        C7(C7_BASE_ALLIGNMENT_POSE),
        C8(C8_BASE_ALLIGNMENT_POSE);
        
        private Pose2d blueBaseAllignmentPose;
        private NodeCoulmn(Pose2d blueBaseAllignmentPose) {
            this.blueBaseAllignmentPose = blueBaseAllignmentPose;
        }

        public Pose2d getBlueBaseAllignmentPose() {
            return blueBaseAllignmentPose;
        }

        public static NodeCoulmn fromOrdinal(int ordinal) {
            for (NodeCoulmn col : NodeCoulmn.values()) {
                if (col.ordinal() == ordinal) {
                    return col;
                }
            }
            return null;
        }
    }

    public enum NodeHeight {
        LOW(0.0),
        MID(0.70),
        HIGH(0.70);

        private double extensionOffset;
        private NodeHeight(double extensionOffset) {
            this.extensionOffset = extensionOffset;
        }

        public double getExtensionOffset() {
            return extensionOffset;
        }
    }

    public enum NodeType {
        HYBRID(0.0),
        CONE(0.0),
        CUBE(0.0);
        private double extensionOffset;
        private NodeType(double extensionOffset) {
            this.extensionOffset = extensionOffset;
        }

        public double getExtensionOffset() {
            return extensionOffset;
        }

        public boolean isGamePieceSupported(GamePieceType gamePieceType) {
            if (this == CUBE) {
                return gamePieceType == GamePieceType.CUBE;
            } else if (this == CONE) {
                return gamePieceType == GamePieceType.CONE;
            } else {
                return true;
            }
        }
    }



}
