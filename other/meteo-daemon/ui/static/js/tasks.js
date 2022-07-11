let curTaskId = 0;
let countParams = 0;

function newTask(){
    gel('task-add-head').style.display  = 'block';
    gel('task-add').style.display = 'block';
    gel('task-edit-head').style.display  = 'none';
    gel('task-edit').style.display = 'none';
    gel('add-task').style.display = 'block';
    gel('task-id').value="";
    gel('task-name').value="";
    gel('task-note').value="";
    clearParams();
}

function editTask(task_id){
    curTaskId = task_id;
    const url = "/verify/schedule/task/get/"+task_id
    JsonGetMethod(url, success_task);
}

function success_task(json){
    clearParams();
    gel('task-add-head').style.display  = 'none';
    gel('task-add').style.display = 'none';
    gel('task-edit-head').style.display  = 'block';
    gel('task-edit').style.display = 'block';
    gel('add-task').style.display = 'block';
    gel('task-id').value=json["id"];
    gel('task-name').value=json["name"];
    gel('task-note').value=json["note"];

    const params = json["params"];
    for(let i=0; i < params.length; i++ ){
        addParam('new-params', true, params[i]["name"]);
    }
}

function clearParams(){
    for(let i = countParams; i >= 1 ; i--) {
        let id = "param_"+i.toString();   
        gel(id).remove();
    }
    gel('add-param').innerHTML = ">>";
    countParams = 0;
}

function GetParams(task_id){
    let params = [];
    for(let i = 1; i <= countParams; i++) {  
        let param_name = gel('param_name_'+i.toString());
        if(param_name.value.length != 0){
            let item = {
                name: param_name.value,
                task_id: task_id,
            };
            params.push(item);
        }
    }
    return params;
}

function addOk(){
    const task_id = gel('task-id').value
    const data = {
        id: task_id,
        name: gel('task-name').value,
        note: gel('task-note').value,
        params: GetParams(task_id),
    };
    const url = "/verify/schedule/task/add"
    JsonPostMethod(url, data, success_reload);
}

function editOk(){ 
    const data = {
        name: gel('task-name').value,
        note: gel('task-note').value,
        params: GetParams(curTaskId),
    };
    const url = "/verify/schedule/task/edit/"+curTaskId
    JsonPutMethod(url, data, success_reload);
}

function delTask(task_id, name){
    let answer = window.confirm("Удалить задачу: '" + name + "'?");
    if(answer){
        const url = "/verify/schedule/task/del/"+task_id
        JsonDeleteMethod(url, success_reload);       
    }
}

function addParam(parent_id, visible, name){
    let el = gel(parent_id);
    countParams = countParams +1;
    let id = countParams.toString();
    name = (name == null) ? "" : name;
    const node = 
    `<div class="field" id="param_` + id +`">
        <label class="label-jobs label-left" for="param-name">Параметр:</label>
        <input id="param_name_`+id+`" class="fld-input task-param-name" placeholder="Наименование" value="`+name+`">
        <i class="mega-octicon octicon-x right edit-icon interacive" data-id="param_`+ 
        id + `" onclick="delParam(this.dataset.id)"></i>
        <i class="mega-octicon octicon-plus right edit-icon interacive" data-id="param_`+
        id + `" onclick="addParam(this.dataset.id, true)"></i>
    </div>`
      
    el.insertAdjacentHTML('afterend', node);
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
            addParam('new-params', true, "", "");
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