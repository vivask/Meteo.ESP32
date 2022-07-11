function ShowForm(el_id){
    el = gel(el_id);
    if(el.style.display != 'block'){
        el.style.display = 'block';
        gel('title').value="";
        gel('content').value="";
    }
}

function addUser(){
    const data = {
        username: gel('username').value,
        password: gel('password').value,
        service: gel('service').value,
    };
    JsonPostMethod("/verify/secure/gituser/add", data, success_reload);
}

function deleteUser(id, service){
    let answer = window.confirm("Удалить пользователя для:" + service + "?");
    if(answer){
        const url = "/verify/secure/gituser/del/"+id
        JsonDeleteMethod(url, success_reload);
    }
}


