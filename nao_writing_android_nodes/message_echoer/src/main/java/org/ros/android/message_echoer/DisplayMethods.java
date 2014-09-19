package org.ros.android.message_echoer;

import android.graphics.Color;
import android.graphics.CornerPathEffect;
import android.graphics.Paint;
import android.graphics.drawable.AnimationDrawable;
import android.graphics.drawable.Drawable;
import android.graphics.drawable.ShapeDrawable;
import android.graphics.drawable.shapes.PathShape;
import android.os.Handler;
import android.util.Log;

import org.ros.android.MessageCallable;
import org.ros.message.Duration;

import java.util.List;

import geometry_msgs.PoseStamped;
import nav_msgs.Path;

/**
 * Methods for displaying things, e.g. converting Path into AnimationDrawableWithEndCallback.
 * Created by Deanna Hood on 01.09.14.
 */
public class DisplayMethods {
    private final java.lang.String TAG = this.getClass().toString();
    private int timeoutDuration_mSecs = -1; //time in ms to leave the trajectory displayed before removing it (negative displays indefinitely)
    private static double PPI_tablet = 298.9; //pixels per inch of android tablet

    private int mDisplayHeight, mDisplayWidth;
    private MessageCallable<Integer, Integer> onShapeDrawingFinishCallable;
    private double mDisplayRate = 1.0;
    public static double MM2INCH = 0.0393701; //number of millimetres in one inch (for conversions)
    private TurnPathIntoAnimation mTurnPathIntoAnimation;

    public DisplayMethods(){
        mTurnPathIntoAnimation = new TurnPathIntoAnimation();
    }

    public TurnPathIntoAnimation getTurnPathIntoAnimation(){
        return mTurnPathIntoAnimation;
    }

    public void setDisplayHeight(int height){
        mDisplayHeight = height;
    }
    public void setDisplayWidth(int width){
        mDisplayWidth = width;
    }
    public void setDisplayRate(double rate){
        mDisplayRate = rate;
    }
    public static double MM2PX(double x){ return x*MM2INCH*PPI_tablet; }
    public static double PX2MM(double x){return x/(PPI_tablet*MM2INCH);}

    public static double M2PX(double x){return (MM2PX(x)*1000.0);}
    public static double PX2M(double x){return PX2MM(x)/1000.0;}

    private ShapeDrawable addPointToShapeDrawablePath(float x, float y, android.graphics.Path path, boolean penUp){
        if(!penUp){
            // add point to path
            path.lineTo(x,y);
        }
        else{
            path.moveTo(x,y);
        }

        // make local copy of path and store in new ShapeDrawable
        android.graphics.Path currPath = new android.graphics.Path(path);

        ShapeDrawable shapeDrawable = new ShapeDrawable();
        shapeDrawable.getPaint().setColor(Color.argb(255,124,163,182));//Color.argb(255,138,205,165));//Color.BLUE);

        shapeDrawable.getPaint().setStyle(Paint.Style.STROKE);
        shapeDrawable.getPaint().setStrokeWidth(10);
        shapeDrawable.getPaint().setStrokeJoin(Paint.Join.ROUND);
        shapeDrawable.getPaint().setStrokeCap(Paint.Cap.ROUND);
        shapeDrawable.getPaint().setPathEffect(new CornerPathEffect(30));
        shapeDrawable.getPaint().setAntiAlias(true);          // set anti alias so it smooths
        shapeDrawable.setIntrinsicHeight(mDisplayHeight);
        shapeDrawable.setIntrinsicWidth(mDisplayWidth);
        shapeDrawable.setBounds(0, 0, mDisplayWidth, mDisplayHeight);

        shapeDrawable.setShape(new PathShape(currPath,mDisplayWidth,mDisplayHeight));

        return shapeDrawable;
    }

    private ShapeDrawable addPointToShapeDrawablePath_quad(float x, float y, float x_next, float y_next, android.graphics.Path path, boolean penUp){
        if(!penUp){
            // add point to path using quadratic bezier curve
            path.quadTo(x,y,(x_next+x)/2,(y_next+y)/2);
        }
        else{
            path.moveTo(x,y);
        }
        // make local copy of path and store in new ShapeDrawable
        android.graphics.Path currPath = new android.graphics.Path(path);

        ShapeDrawable shapeDrawable = new ShapeDrawable();
        shapeDrawable.getPaint().setColor(Color.argb(255,124,163,182));//Color.argb(255,138,205,165));//Color.BLUE);

        shapeDrawable.getPaint().setStyle(Paint.Style.STROKE);
        shapeDrawable.getPaint().setStrokeWidth(10);
        shapeDrawable.getPaint().setStrokeJoin(Paint.Join.ROUND);
        shapeDrawable.getPaint().setStrokeCap(Paint.Cap.ROUND);
        shapeDrawable.getPaint().setPathEffect(new CornerPathEffect(30));
        shapeDrawable.getPaint().setAntiAlias(true);          // set anti alias so it smooths
        shapeDrawable.setIntrinsicHeight(mDisplayHeight);
        shapeDrawable.setIntrinsicWidth(mDisplayWidth);
        shapeDrawable.setBounds(0, 0, mDisplayWidth, mDisplayHeight);

        shapeDrawable.setShape(new PathShape(currPath,mDisplayWidth,mDisplayHeight));

        return shapeDrawable;
    }

    public void setOnAnimationFinishCallable(MessageCallable<Integer, Integer> method){
        onShapeDrawingFinishCallable = method;
    }


    public class TurnPathIntoAnimation implements MessageCallable<Drawable, Path> {
        @Override
        public Drawable call(nav_msgs.Path message) {
            double[] shapeCentre_offset = {00.0,00.0};
            ShapeDrawable blankShapeDrawable = new ShapeDrawable(new PathShape(new android.graphics.Path(), 0, 0));
            blankShapeDrawable.setIntrinsicHeight(mDisplayHeight);
            blankShapeDrawable.setIntrinsicWidth(mDisplayWidth);
            blankShapeDrawable.setBounds(0, 0, mDisplayWidth, mDisplayHeight);

            List<PoseStamped> points = message.getPoses();
            AnimationDrawable animationDrawable = new AnimationDrawable();

            android.graphics.Path trajPath = new android.graphics.Path();
            trajPath.moveTo((float) (M2PX(points.get(0).getPose().getPosition().getX()) + shapeCentre_offset[0]), mDisplayHeight - (float) (M2PX(points.get(0).getPose().getPosition().getY()) + shapeCentre_offset[1]));

            long timeUntilFirstFrame_msecs = Math.round(points.get(0).getHeader().getStamp().totalNsecs() / 1000000.0);
            animationDrawable.addFrame(blankShapeDrawable, (int) (timeUntilFirstFrame_msecs/mDisplayRate));
            int totalTime = (int)timeUntilFirstFrame_msecs;

            for (int i = 0; i < points.size() - 1; i++) //special case for last point/frame of trajectory
            {
                //add new trajectory point onto path and create ShapeDrawable to pass as a frame for animation
                PoseStamped p = points.get(i);
                PoseStamped p_next = points.get(i+1);
                geometry_msgs.Point tx = p.getPose().getPosition();
                geometry_msgs.Point tx_next = p_next.getPose().getPosition();
                boolean penUp = p.getHeader().getSeq() == 1;
                boolean penUp_next = p_next.getHeader().getSeq() == 1;
                ShapeDrawable shapeDrawable;
                if(penUp_next){
                    shapeDrawable = addPointToShapeDrawablePath((float) (shapeCentre_offset[0]+M2PX(tx.getX())), mDisplayHeight - (float) (shapeCentre_offset[1]+M2PX(tx.getY())), trajPath, penUp);
                }else{
                    shapeDrawable = addPointToShapeDrawablePath_quad((float) (shapeCentre_offset[0]+M2PX(tx.getX())), mDisplayHeight - (float) (shapeCentre_offset[1]+M2PX(tx.getY())), (float) (shapeCentre_offset[0]+M2PX(tx_next.getX())), mDisplayHeight - (float) (shapeCentre_offset[1]+M2PX(tx_next.getY())), trajPath, penUp);
                }
                //determine the duration of the frame for the animation
                Duration frameDuration = points.get(i + 1).getHeader().getStamp().subtract(p.getHeader().getStamp()); // take difference between times to get appropriate duration for frame to be displayed

                long dt_msecs = Math.round(frameDuration.totalNsecs() / 1000000.0);
                animationDrawable.addFrame(shapeDrawable, (int) (dt_msecs/mDisplayRate)); //unless the duration is over 2mil seconds the cast is ok
                totalTime+=(int)dt_msecs;
            }
            //cover end case
            PoseStamped p = points.get(points.size() - 1);
            boolean penUp = p.getHeader().getSeq() == 1;
            geometry_msgs.Point tx = p.getPose().getPosition();
            float pointToAdd_x = (float) (M2PX(tx.getX()) + shapeCentre_offset[0]);
            float pointToAdd_y = mDisplayHeight - (float) (M2PX(tx.getY()) + shapeCentre_offset[1]);
            ShapeDrawable shapeDrawable = addPointToShapeDrawablePath(pointToAdd_x, pointToAdd_y, trajPath, penUp);


            if (timeoutDuration_mSecs >= 0)//only display the last frame until timeoutDuration has elapsed
            {
                animationDrawable.addFrame(shapeDrawable, (int) (timeoutDuration_mSecs/mDisplayRate));
                animationDrawable.addFrame(blankShapeDrawable, 0); //stop displaying
            } else { //display last frame indefinitely
                //don't add an extra frame unless necessary because otherwise it will delay the animationFinished message!
                animationDrawable.addFrame(shapeDrawable, 0); //think it will be left there until something clears it so time shouldn't matter
            }
            Log.e(TAG, "Total time (in theory): " + String.valueOf(totalTime));
            animationDrawable.setBounds(0, 0, mDisplayWidth, mDisplayHeight);
            animationDrawable.setOneShot(true); //do not auto-restart the animation

            // Pass our animation drawable to drawable class with callback on finish
            AnimationDrawableWithEndCallback animationDrawableWithEndCallback = new AnimationDrawableWithEndCallback(animationDrawable) {
                @Override
                void onAnimationFinish() {
                    // Animation has finished...
                    if (onShapeDrawingFinishCallable != null) {
                        onShapeDrawingFinishCallable.call(0); //notify that the drawing is finished
                    }
                }
            };
            return animationDrawableWithEndCallback;
        }

    }


    /**
     * Class which allows for a callback when the animation is finished.
     * Useful for publishing a message when a shape has finished being displayed.
     * Taken from http://stackoverflow.com/a/6641321/3441246
     */
    public abstract class AnimationDrawableWithEndCallback extends AnimationDrawable {

        /** Handles the animation callback. */
        Handler mAnimationHandler;

        public AnimationDrawableWithEndCallback(AnimationDrawable aniDrawable) {
    /* Add each frame to our animation drawable */
            for (int i = 0; i < aniDrawable.getNumberOfFrames(); i++) {
                this.addFrame(aniDrawable.getFrame(i), aniDrawable.getDuration(i));
            }
        }

        @Override
        public void start() {
            super.start();
    /*
     * Call super.start() to call the base class start animation method.
     * Then add a handler to call onAnimationFinish() when the total
     * duration for the animation has passed
     */
            mAnimationHandler = new Handler();
            mAnimationHandler.postDelayed(new Runnable() {

                public void run() {
                    onAnimationFinish();
                }
            }, getTotalDuration());

        }

        /**
         * Gets the total duration of all frames.
         *
         * @return The total duration.
         */
        public int getTotalDuration() {

            int iDuration = 0;

            for (int i = 0; i < this.getNumberOfFrames(); i++) {
                iDuration += this.getDuration(i);
            }

            return iDuration;
        }

        /**
         * Called when the animation finishes.
         */
        abstract void onAnimationFinish();
    }


}
