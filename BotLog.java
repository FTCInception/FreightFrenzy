/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package Inception.UltimateGoal;

import android.util.Log;

public class BotLog {

    public final int LOGNONE = 0;
    public final int  LOGDEBUG = 1;
    public final int LOGVERBOSE = 2;
    public final int LOGINFO = 4;
    public final int LOGWARN = 8;

    public int LOGLEVEL = LOGNONE;

    /* Constructor */
    public BotLog() {
    }

    public BotLog(int loglvl) {
        LOGLEVEL = loglvl;
    }

    public void logD(String tag, String msgFormat, Object... args) {
        if ((LOGLEVEL & LOGDEBUG)!=0) {
            Log.d(tag, String.format(msgFormat, args));
        }
    }

    public void logV(String tag, String msgFormat, Object... args) {
        if ((LOGLEVEL & LOGVERBOSE)!=0) {
            Log.v(tag, String.format(msgFormat, args));
        }
    }

    public void logI(String tag, String msgFormat, Object... args) {
        if ((LOGLEVEL & LOGINFO)!=0) {
            Log.i(tag, String.format(msgFormat, args));
        }
    }

    public void logW(String tag, String msgFormat, Object... args) {
        if ((LOGLEVEL & LOGWARN)!=0) {
            Log.w(tag, String.format(msgFormat, args));
        }
    }
}

