function handleTelemetry(msg)
{
    console.log("Telemetry:", msg);

    const data = JSON.parse(msg);

    document.getElementById("roll").innerText =
        data.roll.toFixed(2);

    document.getElementById("pitch").innerText =
        data.pitch.toFixed(2);

    document.getElementById("yaw").innerText =
        data.yaw.toFixed(2);

 
}