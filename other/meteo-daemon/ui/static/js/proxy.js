
function onActivateMaster() {
    JsonPutMethod("/verify/proxy/master/onoff", null, success_reload);
}

function onBlkListMaster() {
    JsonPutMethod("/verify/proxy/master/blklist/onoff", null, success_reload);
}

function onCacheMaster() {
    JsonPutMethod("/verify/proxy/master/cache/onoff", null, success_reload);
}

function onUnlockerMaster() {
    JsonPutMethod("/verify/proxy/master/unlocker/onoff", null, success_reload);
}

function onActivateSlave() {
    JsonPutMethod("/verify/proxy/slave/onoff", null, success_reload);
}

function onBlkListSlave() {
    JsonPutMethod("/verify/proxy/slave/blklist/onoff", null, success_reload);
}

function onCacheSlave() {
    JsonPutMethod("/verify/proxy/slave/cache/onoff", null, success_reload);
}

function onUnlockerSlave() {
    JsonPutMethod("/verify/proxy/slave/blklist/onoff", null, success_reload);
}
