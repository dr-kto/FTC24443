package queue.dicelibrary;

import static queue.dicelibrary.TeamPropLocation.LEFT;
import static queue.dicelibrary.TeamPropLocation.MIDDLE;
import static queue.dicelibrary.TeamPropLocation.RIGHT;

import java.util.List;

public abstract class Action {

    abstract void mirror();

    public static void mirrorActions(List<Action> actions) {
        for(Action action : actions) {
            action.mirror();
        }
    }

    public static double mirrorAngle(double angle) {
        return -angle;
    }

    public static TeamPropLocation mirrorLocation(TeamPropLocation location) {
        if(location == LEFT) {
            return RIGHT;
        }
        else if(location == RIGHT) {
            return LEFT;
        }
        else {
            return MIDDLE;
        }
    }

    public static void mirrorPose(RobotPose pose) {
        pose.y = mirrorY(pose.y);
        pose.heading = mirrorAngle(pose.heading);
    }

    public static double mirrorY(double y) {
        return -y;
    }

}