
function pingHost(ip){
    const url = "/verify/proxy/rest/arp/ping/"+ip
    JsonGetMethod(url, success_reload);
}

function newHost(){
    let el = gel('add-host');
    gel('host-add-head').style.display  = 'block';
    gel('host-add').style.display = 'block';
    gel('host-edit-head').style.display  = 'none';
    gel('host-edit').style.display = 'none';
    el.style.display = 'block';
    gel('hostname').value="";
    gel('ip-address').value="";
    gel('mac-address').value="";
    gel('note').value="";
    window.scrollTo(0, document.body.scrollHeight);
}


function addOk(){
    const data = {
        domain_name: gel('hostname').value,
        ip: gel('ip-address').value,
        mac: gel('mac-address').value,
        note: gel('note').value,
    };
    JsonPutMethod("/verify/proxy/homehost/add", data, success_reload);
}

function editOk(){   
    const data = {
        domain_name: gel('hostname').value,
        ip: gel('ip-address').value,
        mac: gel('mac-address').value,
        note: gel('note').value,
    };
    JsonPostMethod("/verify/proxy/homehost/edit/"+curID, data, success_reload);
}

function delHost(id, name){
    let answer = window.confirm("Удалить хост: '" + name + "'?");
    if(answer){
        JsonDeleteMethod("/verify/proxy/homehost/del/"+id, success_reload);
    }
}

function editHost(id){
    curID = id;
    JsonGetMethod("/verify/proxy/homehost/get/"+id, success_host);
}


function success_host(json){
    gel('host-add-head').style.display  = 'none';
    gel('host-add').style.display = 'none';
    gel('host-edit-head').style.display  = 'block';
    gel('host-edit').style.display = 'block';
    gel('add-host').style.display = 'block';

    gel('hostname').value=json["domain_name"]
    gel('ip-address').value=json["ip"]
    gel('mac-address').value=json["mac"]
    gel('note').value=json["note"]
    window.scrollTo(0, document.body.scrollHeight);
}