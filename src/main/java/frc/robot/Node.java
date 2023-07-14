
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.Optional;

import edu.wpi.first.math.geometry.Pose2d;

 /** Add your docs here. */
public enum Node {
    //-------------------- LEFT GROUP --------------------
    C0_L(NodeCoulmn.C0, NodeHeight.LOW, NodeType.HYBRID),
    C0_M(NodeCoulmn.C0, NodeHeight.MID, NodeType.CONE),
    C0_H(NodeCoulmn.C0, NodeHeight.HIGH, NodeType.CONE),

    C1_L(NodeCoulmn.C1, NodeHeight.LOW, NodeType.HYBRID),
    C1_M(NodeCoulmn.C1, NodeHeight.MID, NodeType.CUBE),
    C1_H(NodeCoulmn.C1, NodeHeight.HIGH, NodeType.CUBE),

    C2_L(NodeCoulmn.C2, NodeHeight.LOW, NodeType.HYBRID),
    C2_M(NodeCoulmn.C2, NodeHeight.MID, NodeType.CONE),
    C2_H(NodeCoulmn.C2, NodeHeight.HIGH, NodeType.CONE),
    //-------------------- CENTER GROUP --------------------
    C3_L(NodeCoulmn.C3, NodeHeight.LOW, NodeType.HYBRID),
    C3_M(NodeCoulmn.C3, NodeHeight.MID, NodeType.CONE),
    C3_H(NodeCoulmn.C3, NodeHeight.HIGH, NodeType.CONE),

    C4_L(NodeCoulmn.C4, NodeHeight.LOW, NodeType.HYBRID),
    C4_M(NodeCoulmn.C4, NodeHeight.MID, NodeType.CUBE),
    C4_H(NodeCoulmn.C4, NodeHeight.HIGH, NodeType.CUBE),

    C5_L(NodeCoulmn.C5, NodeHeight.LOW, NodeType.HYBRID),
    C5_M(NodeCoulmn.C5, NodeHeight.MID, NodeType.CONE),
    C5_H(NodeCoulmn.C5, NodeHeight.HIGH, NodeType.CONE),
    //-------------------- RIGHT GROUP --------------------
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

    public Pose2d getAllignmentPose() {
        return new Pose2d(coulmn.getBlueBaseAllignmentPose().getX() + height.getAllignmentOffset() + type.getAllignmentOffset(), coulmn.getBlueBaseAllignmentPose().getY(), coulmn.getBlueBaseAllignmentPose().getRotation());
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
        C0(new Pose2d()),
        C1(new Pose2d()),
        C2(new Pose2d()),
        C3(new Pose2d()),
        C4(new Pose2d()),
        C5(new Pose2d()),
        C6(new Pose2d()),
        C7(new Pose2d()),
        C8(new Pose2d());
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
        MID(0.0),
        HIGH(0.0);

        private double allignmentOffset;
        private NodeHeight(double allignmentOffset) {
            this.allignmentOffset = allignmentOffset;
        }

        public double getAllignmentOffset() {
            return allignmentOffset;
        }
    }

    public enum NodeType {
        HYBRID(0.0),
        CONE(0.0),
        CUBE(0.0);
        private double allignmentOffset;
        private NodeType(double allignmentOffset) {
            this.allignmentOffset = allignmentOffset;
        }

        public double getAllignmentOffset() {
            return allignmentOffset;
        }
    }



}
