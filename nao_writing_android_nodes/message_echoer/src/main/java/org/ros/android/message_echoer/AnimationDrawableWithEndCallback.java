package org.ros.android.message_echoer;

import android.graphics.drawable.AnimationDrawable;
import android.os.Handler;

/**
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