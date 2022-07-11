
function Signup(){
    const user = gel('username').value;
    const now = new Date();
    let data = {
        ID: cyrb53(user).toString(),
        Password: gel('password').value.toString(),
        Username: user.toString(),
    };
    JsonFetchRequest('/auth/signup', data, success_signup);
}

function success_signup(json){
    const status = json["status"];
    const msg = json["message"];
    if(status == true && msg == 'user created successfully'){
        window.location = "/login";
    }else{
        alert(msg);
    }
}