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