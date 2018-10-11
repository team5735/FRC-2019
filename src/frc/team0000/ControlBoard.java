package frc.team0000;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;

public class ControlBoard implements ControlBoardInterface{
    private final XboxController xboxController;

    private static ControlBoard instance = new ControlBoard();

    public static ControlBoard getInstance() {
        return instance;
    }

    protected ControlBoard() {
        xboxController = new XboxController(0);
    }

    @Override
    public double getThrottle() {
        return xboxController.getY(GenericHID.Hand.kRight);
    }

    @Override
    public double getTurn() {
        return xboxController.getX(GenericHID.Hand.kLeft);
    }

    @Override
    public boolean getQuickTurn() {
        // R1
        return xboxController.getTriggerAxis(GenericHID.Hand.kLeft) > 0.5;
    }

    @Override
    public boolean getAimButton() {
//        return xboxController.getRawButton(8);
        return false;
    }
}
