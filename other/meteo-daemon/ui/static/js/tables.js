let curTableId = "";
let countParams = 0;

function newTable(){
    gel('table-add-head').style.display  = 'block';
    gel('table-add').style.display = 'block';
    gel('table-edit-head').style.display  = 'none';
    gel('table-edit').style.display = 'none';
    gel('add-table').style.display = 'block';
    gel('table-name').value="";
    gel('table-note').value="";
    clearParams();
    addParam('new-params', true, null);
    gel('table-name').focus();
    window.scrollTo(0, document.body.scrollHeight);
}

function editTable(table_id){
    curTableId = table_id;
    const url = "/verify/db/table/get/"+table_id
    JsonGetMethod(url, success_table);
}

function success_table(json){
    clearParams();
    gel('table-add-head').style.display  = 'none';
    gel('table-add').style.display = 'none';
    gel('table-edit-head').style.display  = 'block';
    gel('table-edit').style.display = 'block';
    gel('add-table').style.display = 'block';
    gel('table-name').value=json["name"];
    gel('table-note').value=json["note"];

    const params = json["params"];
    for(let i=0; i < params.length; i++ ){
        addParam('new-params', true, params[i]["sync_type"]);
    }
    gel('table-name').focus();
    window.scrollTo(0, document.body.scrollHeight);
}

function clearParams(){
    for(let i = countParams; i >= 1 ; i--) {
        let id = "param_"+i.toString();   
        gel(id).remove();
    }
    gel('add-param').innerHTML = ">>";
    countParams = 0;
}

function GetParams(table_id){
    let params = [];
    for(let i = 1; i <= countParams; i++) {  
        let param_name = gel('param_name_'+i.toString());
        if(param_name.value.length != 0){
            let item = {
                sync_type: param_name.value,
                table_id: table_id,
            };
            params.push(item);
        }
    }
    return params;
}

function addOk(){
    const table_id = gel('table-name').value
    const data = {
        name: table_id,
        note: gel('table-note').value,
        params: GetParams(table_id),
    };
    const url = "/verify/db/table/add"
    JsonPostMethod(url, data, success_reload);
}

function editOk(){ 
    const table_id = gel('table-name').value
    const data = {
        name: table_id,
        note: gel('table-note').value,
        params: GetParams(table_id),
    };
    const url = "/verify/db/table/edit/"+curTableId
    JsonPutMethod(url, data, success_reload);
}


function delTable(table_id){
    let answer = window.confirm("Удалить задачу: '" + table_id + "'?");
    if(answer){
        const url = "/verify/db/table/del/"+table_id
        JsonDeleteMethod(url, success_reload);       
    }
}


function addParam(parent_id, visible, name){
    let el = gel(parent_id);
    countParams = countParams +1;
    let id = countParams.toString();
    name = (name == null) ? "replace" : name;
    const node = 
    `<div class="field" id="param_` + id +`">
        <label class="label-jobs label-left" for="param-name">Способ обмена:</label>
        <select class="fld-input table-param-name" id="param_name_`+id+`">
            <option value="replace">Замена</option>
            <option value="sync">Синхронизация</option>
        </select>
        <i class="mega-octicon octicon-x right edit-icon interacive" data-id="param_`+ 
        id + `" onclick="delParam(this.dataset.id)"></i>
        <i class="mega-octicon octicon-plus right edit-icon interacive" data-id="param_`+
        id + `" onclick="addParam(this.dataset.id, true)"></i>
    </div>`
    el.insertAdjacentHTML('afterend', node);
    const param_name =  "param_name_"+id
    gel(param_name).value = name;
    if(visible == false){
        gel('param_'+id).style.display = 'none';
    }else{
        gel('add-param').innerHTML = '<<';
    }
}

function delParam(id){
    el = gel(id);
    el.remove();
    countParams = countParams - 1;
    if(countParams == 0){
        gel('add-param').innerHTML = ">>";    
    }
}

function showParams(el){
    if(el.innerHTML === '&gt;&gt;'){
        el.innerHTML = "<<";
        if(countParams == 0){
            addParam('new-params', true, null);
        }else{
            for(let i = 1; i <= countParams; i++) {
                let id = "param_"+i.toString();
                gel(id).style.display = 'block';    
            }
        }
    }else{
        for(let i = 1; i <= countParams; i++) {
            let id = "param_"+i.toString();
            gel(id).style.display = 'none';
        }
        el.innerHTML = ">>";
    }
}