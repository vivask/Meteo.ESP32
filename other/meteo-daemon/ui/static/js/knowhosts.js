
function deleteHost(id, host){
    let answer = window.confirm("Удалить хост:" + host + "?");
    if(answer){
        const url = "/verify/secure/knowhosts/del/"+id
        JsonDeleteMethod(url, success_reload);
    }
}

