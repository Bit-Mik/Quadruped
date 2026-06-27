function handleTelemetry(msg)
{
    console.log("Telemetry:", msg);

    const data = JSON.parse(msg);

    if(data.type == "pid")
    {
        document.getElementById("pidStatus").innerText =
            "PID values applied.";
        return;
    }

    if(data.roll === undefined ||
       data.pitch === undefined ||
       data.yaw === undefined)
    {
        return;
    }

    document.getElementById("roll").innerText =
        data.roll.toFixed(2);

    document.getElementById("pitch").innerText =
        data.pitch.toFixed(2);

    document.getElementById("yaw").innerText =
        data.yaw.toFixed(2);

 
}
