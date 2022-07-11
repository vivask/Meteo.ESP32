function ShowForm(el_id){
    el = gel(el_id);
    if(el.style.display != 'block'){
        el.style.display = 'block';
        gel('title').value="";
        gel('content').value="";
    }
}

function addKey(){
    const data = {
        owner: gel('title').value,
        finger: gel('content').value,
    };
    JsonPostMethod("/verify/secure/gitkey/add", data, success_reload);
}

function deleteKey(id, owner){
    let answer = window.confirm("Удалить ключ:" + owner + "?");
    if(answer){
        const url = "/verify/secure/gitkey/del/"+id
        JsonDeleteMethod(url, success_reload);
    }
}
