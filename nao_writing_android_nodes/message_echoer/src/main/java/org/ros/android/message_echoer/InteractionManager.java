/*
 *
 * Licensed under the Apache License, Version 2.0 (the "License"); you may not
 * use this file except in compliance with the License. You may obtain a copy of
 * the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the
 * License for the specific language governing permissions and limitations under
 * the License.
 */

package org.ros.android.message_echoer;

import android.util.Log;
import org.ros.namespace.GraphName;
import org.ros.node.AbstractNodeMain;
import org.ros.node.ConnectedNode;
import org.ros.node.topic.Publisher;
import java.util.ArrayList;

import geometry_msgs.PointStamped;
import geometry_msgs.PoseStamped;
import nav_msgs.Path;
import std_msgs.Empty;

/**
 * ROS node for capturing and publishing user interactions.
 *
 * @author Deanna Hood
 */

class InteractionManager extends AbstractNodeMain {
    private static final java.lang.String TAG = "InteractionManager";
    private String touchInfoTopicName;
    private ConnectedNode connectedNode;
    private Publisher<PointStamped> touchInfoPublisher;
    private Publisher<PointStamped> gestureInfoPublisher;
    private String gestureInfoTopicName;
    private Publisher<Empty> clearScreenPublisher;
    private String clearScreenTopicName;
    private Publisher<Path> userDrawnShapePublisher;
    private String userDrawnShapeTopicName;

    public void setTouchInfoTopicName(String topicName) {
        this.touchInfoTopicName = topicName;
    }
    public void setGestureInfoTopicName(String topicName) { this.gestureInfoTopicName = topicName; }
    public void setClearScreenTopicName(String topicName) {
        this.clearScreenTopicName = topicName;
    }
    public void setUserDrawnShapeTopicName(String topicName) {
        this.userDrawnShapeTopicName = topicName;
    }

    @Override
    public GraphName getDefaultNodeName() {
        return GraphName.of("touch_publisher");
    }

    @Override
    public void onStart(final ConnectedNode connectedNode) {
         this.connectedNode = connectedNode;
        this.touchInfoPublisher =
                connectedNode.newPublisher(touchInfoTopicName, geometry_msgs.PointStamped._TYPE);
        this.touchInfoPublisher.setLatchMode(false);
        this.gestureInfoPublisher =
                connectedNode.newPublisher(gestureInfoTopicName, geometry_msgs.PointStamped._TYPE);
        this.gestureInfoPublisher.setLatchMode(false);
        this.clearScreenPublisher =
                connectedNode.newPublisher(clearScreenTopicName, Empty._TYPE);
        this.userDrawnShapePublisher =
                connectedNode.newPublisher(userDrawnShapeTopicName, Path._TYPE);

    }

    public void publishTouchInfoMessage(double x, double y) {

        geometry_msgs.PointStamped pointStamped = touchInfoPublisher.newMessage();
        pointStamped.getHeader().setStamp(connectedNode.getCurrentTime());
        pointStamped.getPoint().setX(x);
        pointStamped.getPoint().setY(y);

        touchInfoPublisher.publish(pointStamped);

    }
    public void publishGestureInfoMessage(double x, double y) {

        geometry_msgs.PointStamped pointStamped = gestureInfoPublisher.newMessage();
        pointStamped.getHeader().setStamp(connectedNode.getCurrentTime());
        pointStamped.getPoint().setX(x);
        pointStamped.getPoint().setY(y);

        gestureInfoPublisher.publish(pointStamped);

    }
    public void publishClearScreenMessage(){
        Log.e(TAG, "Publishing clear screen request");
        Empty message = clearScreenPublisher.newMessage();
        clearScreenPublisher.publish(message);
    }

    public void publishUserDrawnShapeMessage(ArrayList<double[]> points){
        Log.e(TAG, "Publishing user-drawn shape");
        nav_msgs.Path message = userDrawnShapePublisher.newMessage();
        message.getHeader().setStamp(connectedNode.getCurrentTime());

        for(double[] point : points){
            PoseStamped poseStamped = connectedNode.getTopicMessageFactory().newFromType(PoseStamped._TYPE);

            poseStamped.getPose().getPosition().setX(point[0]);
            poseStamped.getPose().getPosition().setY(point[1]);

            message.getPoses().add(poseStamped);
        }
        userDrawnShapePublisher.publish(message);
    }

    public void publishUserDrawnMessageMessage(ArrayList< ArrayList<double[]> > userDrawnMessage){
        Log.e(TAG, "Publishing user-drawn message");
        for(ArrayList<double[]> stroke : userDrawnMessage){
            publishUserDrawnShapeMessage(stroke);
        }
        publishUserDrawnShapeMessage(new ArrayList<double[]>()); //publish empty stroke to show message is done
    }
}