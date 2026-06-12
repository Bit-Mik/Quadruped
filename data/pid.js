function updateRollPID()
{
    const kp =
        document.getElementById("kp_roll").value;

    const ki =
        document.getElementById("ki_roll").value;

    const kd =
        document.getElementById("kd_roll").value;

    ws.send(JSON.stringify({
        type: "pid",
        axis: "roll",
        kp: kp,
        ki: ki,
        kd: kd
    }));
}