
function Login(){
    const user = gel('username').value;
    const id = cyrb53(user).toString();
    let data = {
        ID: id,
        Password: gel('password').value.toString(),
        Username: user.toString(),
    };
    JsonFetchRequest('/auth/login', data, success_login, id);
}

function success_login(json, id){
    const status = json["status"];
    const msg = json["message"];
    if(status == true && msg == 'Successfully logged in'){
        let data = json["data"];
        localStorage.setItem("access_token", data["access_token"]);
        localStorage.setItem("refresh_token", data["refresh_token"]);
        localStorage.setItem("user_id", id);
        gel('login').style.display = 'none';
        gel('logout').style.display = 'block';
        window.location = "/";
    }else{
        alert(msg);
    }
}