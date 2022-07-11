let curID = 0;

function newHost(){
    let el = gel('add-host');
    gel('host-add-head').style.display  = 'block';
    gel('host-add').style.display = 'block';
    gel('host-edit-head').style.display  = 'none';
    gel('host-edit').style.display = 'none';
    el.style.display = 'block';
    gel('hostname').value="";
    gel('note').value="";
    gel('access-list').value="tovpn";
}


function addOk(){
    const data = {
        name: gel('hostname').value,
        note: gel('note').value,
        list_id: gel('access-list').value,
    };
    showWaitWin('popup-wait');
    JsonPostMethod("/verify/proxy/manual/add", data, success);
}

function editOk(){   
    const data = {
        name: gel('hostname').value,
        note: gel('note').value,
        list_id: gel('access-list').value,
    };
    showWaitWin('popup-wait');
    JsonPostMethod("/verify/proxy/manual/edit/"+curID, data, success);
}

function delHost(id, name){
    let answer = window.confirm("Удалить хост: '" + name + "'?");
    if(answer){
        showWaitWin('popup-wait');
        JsonDeleteMethod("/verify/proxy/manual/del/"+id, success);
    }
}

function success(json){
    window.location=location; 
    hideWaitWin('popup-wait');
}

function editHost(id){
    curID = id;
    JsonGetMethod("/verify/proxy/manual/get/"+id, success_host);
}


function success_host(json){
    gel('host-add-head').style.display  = 'none';
    gel('host-add').style.display = 'none';
    gel('host-edit-head').style.display  = 'block';
    gel('host-edit').style.display = 'block';
    gel('add-host').style.display = 'block';

    gel('hostname').value=json["name"]
    gel('note').value=json["note"]
    gel('access-list').value=json["list_id"]
}

