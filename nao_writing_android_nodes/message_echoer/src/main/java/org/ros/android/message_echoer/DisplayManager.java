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

import android.content.Context;
import android.graphics.drawable.AnimationDrawable;
import android.graphics.drawable.Drawable;
import android.graphics.drawable.LayerDrawable;
import android.graphics.drawable.ShapeDrawable;
import android.graphics.drawable.shapes.PathShape;
import android.util.AttributeSet;
import android.util.Log;
import android.widget.ImageView;
import org.ros.android.MessageCallable;
import org.ros.message.Duration;
import org.ros.message.MessageListener;
import org.ros.namespace.GraphName;
import org.ros.node.ConnectedNode;
import org.ros.node.Node;
import org.ros.node.NodeMain;
import org.ros.node.parameter.ParameterTree;
import org.ros.node.topic.Publisher;
import org.ros.node.topic.Subscriber;

import java.lang.String;

import nav_msgs.Path;
import std_msgs.*;

/**
 * ROS node for displaying things to user, e.g. incoming nav_msgs Path messages in a LayerDrawable.
 * Based on sample ROSAndroid code under apache-2.0 license.
 * @author deanna.m.hood@gmail.com (Deanna Hood).
 */
public class DisplayManager extends ImageView implements NodeMain {
    private static final java.lang.String TAG = "DisplayManager";
    private static final boolean WAIT_TO_SYNC_TRAJ_DEFAULT = true; //if using a simulated time, this should be true, so don't wait until the requested start time of shape
    private boolean WAIT_TO_SYNC_TRAJ;

    private String topicName;
    private String messageType;
    private MessageCallable<Drawable, nav_msgs.Path> drawableCallable;

    private String clearScreenTopicName;
    private MessageCallable<Integer, Integer> clearScreenCallable;
    private Publisher<std_msgs.String> finishedShapePublisher;
    private String finishedShapeTopicName;
    private Publisher<std_msgs.Empty> clearWatchdogPublisher;

    public DisplayManager(Context context) {
        super(context);
    }

    public DisplayManager(Context context, AttributeSet attrs) {
        super(context, attrs);
    }

    public DisplayManager(Context context, AttributeSet attrs, int defStyle) {
        super(context, attrs, defStyle);
    }

    public void setTopicName(String topicName) {
        this.topicName = topicName;
    }

    public void setClearScreenTopicName(String topicName) {
        this.clearScreenTopicName = topicName;
    }

    public void setFinishedShapeTopicName(String topicName) {
        this.finishedShapeTopicName = topicName;
    }

    public void setMessageType(String messageType) {
        this.messageType = messageType;
    }

    /**
     * Set which method will be called when a Path message is received, to convert it into a Drawable
     */
    public void setMessageToDrawableCallable(MessageCallable<Drawable, nav_msgs.Path> callable) {
        this.drawableCallable = callable;
    }

    /**
     * Set which method will be called when a clear screen message is received (after clearing DisplayManager's View).
     */
    public void setClearScreenCallable(MessageCallable<Integer, Integer > callable) {
        this.clearScreenCallable = callable;
    }


    @Override
    public GraphName getDefaultNodeName() {
        return GraphName.of("ros_path_view");
    }

    @Override
    public void onStart(final ConnectedNode connectedNode) {
        ParameterTree param = connectedNode.getParameterTree();
        WAIT_TO_SYNC_TRAJ = param.getBoolean("wait_to_sync_traj", WAIT_TO_SYNC_TRAJ_DEFAULT);
        Log.e(TAG, "ROSparam for wait_to_sync_traj exists on server: " + Boolean.toString(param.has("wait_to_sync_traj")));

        Subscriber<nav_msgs.Path> subscriber = connectedNode.newSubscriber(topicName, messageType);
        subscriber.addMessageListener(new MessageListener<nav_msgs.Path>() {
            @Override
            public void onNewMessage(final nav_msgs.Path message) {
                if (drawableCallable != null) {

                    if(message.getPoses().size()!=0){//prevent drawing the stop bit
                        post(new UponReceivingMessage(connectedNode, message));
                    }
                }
                postInvalidate();
            }
        });


        Subscriber<Empty> clearScreenSubscriber = connectedNode.newSubscriber(clearScreenTopicName, Empty._TYPE);
        clearScreenSubscriber.addMessageListener(new MessageListener<Empty>() {
            @Override
            public void onNewMessage(final Empty message) {
                post(new Runnable() {
                    @Override
                    public void run() {
                        Log.e(TAG,"Got clear screen request.");
                        ShapeDrawable blankShapeDrawable = new ShapeDrawable(new PathShape(new android.graphics.Path(), 0, 0));
                        blankShapeDrawable.setIntrinsicHeight(getHeight());
                        blankShapeDrawable.setIntrinsicWidth(getWidth());
                        blankShapeDrawable.setBounds(0, 0, getWidth(), getHeight());
                        setImageDrawable(blankShapeDrawable);
                    }
                });
                if (clearScreenCallable != null) {
                    clearScreenCallable.call(0); //give the opportunity for any other objects to act
                }
            }
        });
        this.finishedShapePublisher =
                connectedNode.newPublisher(finishedShapeTopicName, std_msgs.String._TYPE);
    }

    public void publishShapeFinishedMessage(){
        Log.e(TAG, "Publishing shape finished message.");
        std_msgs.String message = finishedShapePublisher.newMessage();
        message.setData(String.valueOf(true));
        finishedShapePublisher.publish(message);
    }

    public void publishWatchdogClearMessage(){
        //Log.e(TAG, "Publishing clear watchdog message.");
        std_msgs.Empty message = clearWatchdogPublisher.newMessage();
        clearWatchdogPublisher.publish(message);
    }

    class UponReceivingMessage implements Runnable {
        private ConnectedNode mConnectedNode;
        private Path mMessage;
        public UponReceivingMessage(ConnectedNode connectedNode, Path message){
            mConnectedNode = connectedNode;
            mMessage = message;
        }
        @Override
        public void run() {
            Log.e(TAG, "got a message at " + mConnectedNode.getCurrentTime().toString());

            AnimationDrawable drawable = (AnimationDrawableWithEndCallback)drawableCallable.call(mMessage);

            LayerDrawable layerDrawable;
            Drawable currentDrawable = getDrawable();
            if(currentDrawable == null){ //first drawable
                setImageDrawable(drawable);
            }else{
                if(currentDrawable instanceof LayerDrawable){ //already a layerDrawable
                    //add new layer
                    int numExistingLayers = ((LayerDrawable) currentDrawable).getNumberOfLayers();
                    Drawable[] layers = new Drawable[numExistingLayers+1];
                    for(int i = 0; i<numExistingLayers-1; i++){
                        layers[i] = ((LayerDrawable) currentDrawable).getDrawable(i);
                    }
                    AnimationDrawable previousAnimation = ((AnimationDrawable)((LayerDrawable) currentDrawable).getDrawable(numExistingLayers-1));
                    layers[numExistingLayers-1] = previousAnimation.getFrame(previousAnimation.getNumberOfFrames()-1); //prevent re-animation
                    layers[numExistingLayers] = drawable;
                    layerDrawable = new LayerDrawable(layers);
                    setImageDrawable(layerDrawable);
                }
                else{
                    //turn into layerDrawable
                    Drawable[] layers = new Drawable[2];
                    layers[0] = currentDrawable;
                    layers[1] = drawable;
                    layerDrawable = new LayerDrawable(layers);
                    setImageDrawable(layerDrawable);
                }
            }

            // wait until appropriate time to start animation
            Duration delay = mMessage.getHeader().getStamp().subtract(mConnectedNode.getCurrentTime());
            if(WAIT_TO_SYNC_TRAJ){
                try{Thread.sleep(Math.round((delay.totalNsecs() / 1000000.0)));}
                catch(InterruptedException e){
                    Log.e(TAG, "InterruptedException: " + e.getMessage());
                }
                catch(IllegalArgumentException e){
                    Log.e(TAG, "IllegalArgumentException: " + e.getMessage());
                }
            }
            Log.e(TAG, "executing message at " + mConnectedNode.getCurrentTime().toString());

            drawable.start(); //start animation (and call callback when finished)
        }
    }

    @Override
    public void onShutdown(Node node) {
    }

    @Override
    public void onShutdownComplete(Node node) {
    }

    @Override
    public void onError(Node node, Throwable throwable) {
    }
}
