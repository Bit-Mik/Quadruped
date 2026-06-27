function readNumber(id)
{
    return parseFloat(
        document.getElementById(id).value
    );
}

function applyPID()
{
    ws.send(JSON.stringify({
        type: "pid",

        roll: {
            kp: readNumber("kp_roll"),
            ki: readNumber("ki_roll"),
            kd: readNumber("kd_roll")
        },

        pitch: {
            kp: readNumber("kp_pitch"),
            ki: readNumber("ki_pitch"),
            kd: readNumber("kd_pitch")
        },

        rollDeadband: readNumber("roll_deadband"),
        pitchDeadband: readNumber("pitch_deadband"),
        maxRollCorr: readNumber("max_roll_corr"),
        maxPitchCorr: readNumber("max_pitch_corr")
    }));

    document.getElementById("pidStatus").innerText =
        "Sending PID values...";
}
