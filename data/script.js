const ws = new WebSocket(
    "ws://" + location.host + "/ws"
);

ws.onopen = () =>
{
    console.log("connected");
};

ws.onmessage = (event) =>
{
    console.log("RX:", event.data);

    handleTelemetry(event.data);
};

function sendCmd(cmd)
{
    ws.send(cmd);
}
function test()
{
    ws.send("hello");
}

function moveServo()
{
    ws.send(JSON.stringify({
        type: "servo",
        index: parseInt(
            document.getElementById("servoIndex").value
        ),
        angle: parseFloat(
            document.getElementById("servoAngle").value
        )
    }));
}

function moveLeg()
{
    ws.send(JSON.stringify({
        type: "leg",
        leg:
            document.getElementById("leg").value,

        x: parseFloat(
            document.getElementById("x").value
        ),

        y: parseFloat(
            document.getElementById("y").value
        ),

        z: parseFloat(
            document.getElementById("z").value
        )
    }));
}