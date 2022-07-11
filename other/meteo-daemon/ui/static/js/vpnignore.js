function delHost(name){
    let answer = window.confirm("Удалить хост: '" + name + "'?");
    if(answer){
        showWaitWin('popup-wait');
        JsonDeleteMethod("/verify/proxy/ignore/del/"+name, success);
    }
}

function Restore(name){
    let answer = window.confirm("Вернуть хост: '" + name + "' в VPN тунель?");
    if(answer){
        showWaitWin('popup-wait');
        JsonPutMethod("/verify/proxy/ignore/restore/"+name, null, success);
    }
}

function success(){
    document.location.reload();
    hideWaitWin('popup-wait');
}