let masterList = [];
let slaveList = [];

function chbMasterClick(chbox, id){
    if (chbox.checked) {
        masterList.push(id);
	}
	else {
        let index = masterList.indexOf(id);
        if (index !== -1) {
            masterList.splice(index, 1);
        }         
	}    
    if (masterList.length > 0){
        gel('master-sync').disabled = false;
    }else{
        gel('master-sync').disabled = true;
    }
}

function chbSlaveClick(chbox, id){
    if (chbox.checked) {
        slaveList.push(id);
	}
	else {
        let index = slaveList.indexOf(id);
        if (index !== -1) {
            slaveList.splice(index, 1);
        }         
	}    
    if (slaveList.length > 0){
        gel('slave-sync').disabled = false;
    }else{
        gel('slave-sync').disabled = true;
    }
}

function SyncMasterTable(table_id){
    let answer = false
    if (gel('master-select-'+table_id).value == "sync") {
        answer = window.confirm("Синхронизировать данные в таблицах '" + table_id + "' серверов master и slave?");
    }else{
        answer = window.confirm("Заменить все данные в таблице '" + table_id + "' из сервера master в сервере slave?");
    }
    if(answer){
        showWaitWin('popup-wait');
        syncMasterTable(table_id);       
    }
}

function syncMasterTable(table_id){
    const select_id = "master-select-"+table_id;
    const syncType = gel(select_id).value;
    const url="/verify/db/rest/"+syncType+"/"+table_id+"/master";
    JsonPutMethod(url, null, success_sync_table);
}

function SyncSlaveTable(table_id){
    let answer = false
    if (gel('slave-select-'+table_id).value == "sync") {
        answer = window.confirm("Синхронизировать данные в таблицах '" + table_id + "' серверов master и slave?");
    }else{
        answer = window.confirm("Заменить все данные в таблице '" + table_id + "' из сервера slave в сервере master?");
    }
    if(answer){
        showWaitWin('popup-wait');
        syncSlaveTable(table_id);       
    }
}

function syncSlaveTable(table_id){
    const select_id = "slave-select-"+table_id;
    const syncType = gel(select_id).value;
    const url="/verify/db/rest/"+syncType+"/"+table_id+"/slave";
    JsonPutMethod(url, null, success_sync_table);
}

function SyncMasterTables() {
    let answer = window.confirm("Заменить все данные в выбранных таблицах из сервера master в сервере slave?");
    if(answer){
        showWaitWin('popup-wait');
        syncMasterTables(table_id);       
    }
}

function syncMasterTables() {
    for(let i=0; i < masterList.length; i++ ){
        syncMasterTable(masterList[i])
    }
}

function SyncSlaveTables() {
    let answer = window.confirm("Заменить все данные в выбранных таблицах из сервера slave в сервере master?");
    if(answer){
        showWaitWin('popup-wait');
        syncSlaveTables(table_id);       
    }
}

function syncSlaveTables() {
    for(let i=0; i < slaveList.length; i++ ){
        syncSlaveTable(slaveList[i])
    }
}

function success_sync_table(json){
    hideWaitWin('popup-wait');
}