// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.PS4Controller.Button;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.ScoreingConstants;
import frc.robot.Node.NodeCoulmn;
import frc.robot.Node.NodeHeight;

import static frc.robot.Constants.OperatorConstants.*;

import java.util.Optional;

/** Add your docs here. */
public class OperatorControlPad {
    private final GenericHID hid;
    private final JoystickButton leftNodeGroupButton, centerNodeGroupButton, rightNodeGroupButton;
    private final JoystickButton leftHighNodeButton, centerHighNodeButton, rightHighNodeButton;
    private final JoystickButton leftMidNodeButton, centerMidNodeButton, rightMidNodeButton;
    private final JoystickButton leftLowNodeButton, centerLowNodeButton, rightLowNodeButton;
    private final JoystickButton intakeGroundCubeButton, intakeGroundConeButton;
    private final JoystickButton intakeSingleSubstationCubeButton, intakeSingleSubstationConeButton;
    private final JoystickButton intakeDoubleSubstationCubeButton, intakeDoubleSubstationConeButton;
    private final JoystickButton elevatorStowButton;
    private final JoystickButton scrollClick;
    private final JoystickButton ejectGamePieceButton, rollerExtakeButton;
    private final Trigger leftNodeSelectedTrigger, centerNodeSelectedTrigger, rightNodeSelectedTrigger;
    private final Trigger highNodeSelectedTrigger, midNodeSelectedTrigger, lowNodeSelectedTrigger;
    private final Trigger nodeHeightSelectedTrigger, nodeCoulmnSelectedTrigger;
    private final Trigger scoringCommandRequestTrigger, nodeGroupSelectedTrigger, inGroupNodeSelectedTrigger;
    public OperatorControlPad(int port) {
        hid = new GenericHID(OPERATOR_PAD_PORT);
        leftNodeGroupButton = new JoystickButton(hid, 0);
        centerNodeGroupButton = new JoystickButton(hid, 0);
        rightNodeGroupButton = new JoystickButton(hid, 0);

        leftHighNodeButton = new JoystickButton(hid, port);
        leftMidNodeButton = new JoystickButton(hid, port);
        leftLowNodeButton = new JoystickButton(hid, port);

        centerHighNodeButton = new JoystickButton(hid, port);
        centerMidNodeButton = new JoystickButton(hid, port);
        centerLowNodeButton = new JoystickButton(hid, port);

        rightHighNodeButton = new JoystickButton(hid, port);
        rightMidNodeButton = new JoystickButton(hid, port);
        rightLowNodeButton = new JoystickButton(hid, port);


      
        leftNodeSelectedTrigger = leftHighNodeButton.or(leftMidNodeButton).or(leftLowNodeButton);
        centerNodeSelectedTrigger = centerHighNodeButton.or(centerMidNodeButton).or(centerLowNodeButton);
        rightNodeSelectedTrigger = rightHighNodeButton.or(rightMidNodeButton).or(rightLowNodeButton);
        nodeCoulmnSelectedTrigger = leftNodeSelectedTrigger.or(centerNodeSelectedTrigger).or(rightNodeSelectedTrigger);

        highNodeSelectedTrigger = leftHighNodeButton.or(centerHighNodeButton).or(rightHighNodeButton);
        midNodeSelectedTrigger = leftMidNodeButton.or(centerMidNodeButton).or(rightMidNodeButton);
        lowNodeSelectedTrigger = leftLowNodeButton.or(centerLowNodeButton).or(rightLowNodeButton);
        nodeHeightSelectedTrigger = highNodeSelectedTrigger.or(midNodeSelectedTrigger).or(lowNodeSelectedTrigger);

        nodeGroupSelectedTrigger = leftNodeGroupButton.or(centerNodeGroupButton).or(rightNodeGroupButton);
        inGroupNodeSelectedTrigger = nodeCoulmnSelectedTrigger.and(nodeHeightSelectedTrigger);

        scoringCommandRequestTrigger = nodeGroupSelectedTrigger.and(inGroupNodeSelectedTrigger);
    }

    public JoystickButton getLeftNodeGroupButton() {
        return leftNodeGroupButton;
    }

    public JoystickButton getCenterNodeGroupButton() {
        return centerNodeGroupButton;
    }

    public JoystickButton getRightNodeGroupButton() {
        return rightNodeGroupButton;
    }

    public JoystickButton getLeftHighNodeButton() {
        return leftHighNodeButton;
    }

    public JoystickButton getCenterHighNodeButton() {
        return centerHighNodeButton;
    }

    public JoystickButton getRightHighNodeButton() {
        return rightHighNodeButton;
    }

    public JoystickButton getLeftMidNodeButton() {
        return leftMidNodeButton;
    }

    public JoystickButton getCenterMidNodeButton() {
        return centerMidNodeButton;
    }

    public JoystickButton getRightMidNodeButton() {
        return rightMidNodeButton;
    }

    public JoystickButton getLeftLowNodeButton() {
        return leftLowNodeButton;
    }

    public JoystickButton getCenterLowNodeButton() {
        return centerLowNodeButton;
    }

    public JoystickButton getRightLowNodeButton() {
        return rightLowNodeButton;
    }

    public Trigger getScoringCommandRequestTrigger() {
        return scoringCommandRequestTrigger;
    }

    public JoystickButton getElevatorStowButton() {
        return elevatorStowButton;
    }

    public JoystickButton getIntakeGroundConeButton() {
        return intakeGroundConeButton;
    }

    public JoystickButton getIntakeGroundCubeButton() {
        return intakeGroundCubeButton;
    }

    public JoystickButton getIntakeSingleSubstationCubeButton() {
        return intakeSingleSubstationCubeButton;
    }

    public JoystickButton getIntakeSingleSubstationConeButton() {
        return intakeSingleSubstationConeButton;
    }

    public JoystickButton getIntakeDoubleSubstationConeButton() {
        return intakeDoubleSubstationConeButton;
    }

    public JoystickButton getIntakeDoubleSubstationCubeButton() {
        return intakeDoubleSubstationCubeButton;
    }

    public JoystickButton getEjectGamePieceButton() {
        return ejectGamePieceButton;
    }

    public JoystickButton getRollerExtakeButton() {
        return rollerExtakeButton;
    }


    public JoystickButton getScrollClick() {
        return scrollClick;
    }

    public Optional<Node> getSelectedScoringNode() {
        int colOrd = 0;
        NodeHeight height = NodeHeight.HIGH;
        Optional<Alliance> ally = AllianceManager.getAlliance();
        if (scoringCommandRequestTrigger.getAsBoolean() && ally.isPresent()) {
            if (ally.get() == Alliance.Blue) {

                if (leftNodeGroupButton.getAsBoolean()) {
                    colOrd = ScoreingConstants.BLUE_LEFT_NODE_GROUP_CENTRAL_COULMN_ORDINAL;
                } else if (centerNodeGroupButton.getAsBoolean()) {
                    colOrd = ScoreingConstants.BLUE_CENTER_NODE_GROUP_CENTRAL_COULMN_ORDINAL;
                } else if (rightNodeGroupButton.getAsBoolean()) {
                    colOrd = ScoreingConstants.BLUE_RIGHT_NODE_GROUP_CENTRAL_COULMN_ORDINAL;
                } else {
                    return Optional.empty();
                }

                if (leftNodeSelectedTrigger.getAsBoolean()) {
                    colOrd--;
                } else if (rightNodeSelectedTrigger.getAsBoolean()) {
                    colOrd++;
                } else {
                    return Optional.empty();
                }

            } else {
                if (leftNodeGroupButton.getAsBoolean()) {
                    colOrd = ScoreingConstants.RED_LEFT_NODE_GROUP_CENTRAL_COULMN_ORDINAL;
                } else if (centerNodeGroupButton.getAsBoolean()) {
                    colOrd = ScoreingConstants.RED_CENTER_NODE_GROUP_CENTRAL_COULMN_ORDINAL;
                } else if (rightNodeGroupButton.getAsBoolean()) {
                    colOrd = ScoreingConstants.RED_RIGHT_NODE_GROUP_CENTRAL_COULMN_ORDINAL;
                } else {
                    return Optional.empty();
                }

                if (leftNodeSelectedTrigger.getAsBoolean()) {
                    colOrd++;
                } else if (rightNodeSelectedTrigger.getAsBoolean()) {
                    colOrd--;
                } else {
                    return Optional.empty();
                }
            }
            

            if (highNodeSelectedTrigger.getAsBoolean()) {
                height = NodeHeight.HIGH;
            } else if (midNodeSelectedTrigger.getAsBoolean()) {
                height = NodeHeight.MID;
            } else if (lowNodeSelectedTrigger.getAsBoolean()) {
                height = NodeHeight.LOW;
            } else {
                return Optional.empty();
            }

            return Node.fromCoulmnAndHeight(NodeCoulmn.fromOrdinal(colOrd), height);
        }
        return Optional.empty();
        
    }





}
