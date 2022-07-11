function delHost(name){
    let answer = window.confirm("Удалить хост: '" + name + "'?");
    if(answer){
        showWaitWin('popup-wait');
        JsonDeleteMethod("/verify/proxy/auto/del/"+name, success);
    }
}

function Ignor(name){
    let answer = window.confirm("Исключить хост: '" + name + "' из VPN тунеля?");
    if(answer){
        showWaitWin('popup-wait');
        JsonPutMethod("/verify/proxy/ignore/add/"+name, null, success);
    }
}

function success(){
    document.location.reload()
    hideWaitWin('popup-wait');
}