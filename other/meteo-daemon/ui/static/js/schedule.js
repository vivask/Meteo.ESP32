
let curJobId = 0;
let countParams = 0;

let Period1 = new Map([['one', 'Единоразово'], ['sec', 'Секунд'], ['min', 'Минут'], ['hour', 'Часов'], ['day', 'Дней'], ['week', 'Недель'], ['day_of_week', 'День недели'], ['month', 'Месяцев']]);
let Period2 = new Map([['one', 'Единоразово'], ['sec', 'Секунду'], ['min', 'Минуту'], ['hour', 'Час'], ['day', 'День'], ['week', 'Неделю'], ['day_of_week', 'День недели'], ['month', 'Месяц']]);
let Period3 = new Map([['one', 'Единоразово'], ['sec', 'Секунды'], ['min', 'Минуты'], ['hour', 'Часа'], ['day', 'Дня'], ['week', 'Недели'], ['day_of_week', 'День недели'], ['month', 'Месяца']]);


function newJob(){
    let el = gel('add-task');
    gel('task-add-head').style.display  = 'block';
    gel('task-add').style.display = 'block';
    gel('task-edit-head').style.display  = 'none';
    gel('task-edit').style.display = 'none';
    el.style.display = 'block';
    gel('new-verbose').checked = false;
    gel('note').value="";
    gel('val-day').value="";
    gel('val-num').value="";
    gel('time').value="";
    gel('date').value="";
    gel('num-val').style.display = 'block';
    gel('day-val').style.display = 'none';
    el = gel('period-num');
    el.selectedIndex = 0;
    doChangeTask();
    gel('lb0').innerHTML = 'Повторять каждую';
    validate_num();
    gel('note').focus();
    window.scrollTo(0, document.body.scrollHeight);
}


function GetParams() {
    let params = [];
    for(let i = 1; i <= countParams; i++) {  
        let param_name = gel('param_name_'+i.toString());
        let param_value = gel('param_value_'+i.toString());
        if(param_name.value.length != 0){
            let item = {
                name: param_name.value,
                value: param_value.value,
            };
            params.push(item);
        }
    }
    return params
}

function getVerbose(){
    let verbose = 0
    if(gel('task-add').style.display == 'block'){
        verbose = (gel('new-verbose').checked) ? 1 : 0
    }else{
        verbose = (gel('verbose').checked) ? 1 : 0
    }  
    return parseInt(verbose)
}

function GetJobData(job_id){
    let task = gel('task');
    let executor = gel('executor');
    let period, val, day_id;
    if ( gel('num-val').style.display == 'block' ){
        period = gel('period-num');
        val = gel('val-num').value;
        day_id = '';
    }else{
        period = gel('period-day');
        val = gel('val-day').value;
        let days = gel('days');
        day_id = days.options[days.selectedIndex].value
    }

    let state_id = "active-job-"+job_id.toString()
    let state = true
    if(job_id != 0){
        state = gel(state_id).dataset.active
    }

    let period_val = period.options[period.selectedIndex].value;

    const data = {
        note: gel('note').value.toString(),
        active: (state) ? parseInt(1) : parseInt(0),
        value: parseInt(val),
        time: gel('time').value,
        date: gel('date').value,
        verbose: getVerbose(),
        executor_id: executor.options[executor.selectedIndex].value.toString(),
        task_id: task.options[task.selectedIndex].value.toString(),
        period_id: period_val.toString(),
        day_id: parseInt(day_id),
        params: GetParams(),
    };

    return data;
}

function addOk(){
    showWaitWin('popup-wait');
    const url = "/verify/schedule/job/add"
    const data = GetJobData(0);
    JsonPostMethod(url, data, reload_success);
}

function editOk(){   
    showWaitWin('popup-wait');
    const data = GetJobData(curJobId);
    const url = "/verify/schedule/job/edit/"+curJobId
    JsonPutMethod(url, data, reload_success);
}

function reload_success(json){
    hideWaitWin('popup-wait');
    window.location=location; 
}


function delJob(job_id, name){
    let answer = window.confirm("Удалить задачу: '" + name + "'?");
    if(answer){
        showWaitWin('popup-wait');
        const url = "/verify/schedule/job/del/"+job_id
        JsonDeleteMethod(url, reload_success);       
    }
}

function editJob(job_id){
    curJobId = job_id;
    const url = "/verify/schedule/job/get/"+job_id
    JsonGetMethod(url, success_job, job_id);
}

function success_job(json, job_id){
    clearParams();
    gel('task-add-head').style.display  = 'none';
    gel('task-add').style.display = 'none';
    gel('task-edit-head').style.display  = 'block';
    gel('task-edit').style.display = 'block';
    gel('add-task').style.display = 'block';
    gel('verbose').checked = (json["verbose"] == 1);
    gel('note').value = json["note"];
    gel('task').value = json["task_id"];
    gel('executor').value = json["executor_id"];

    if(json["period_id"] != 'day_of_week') {
        gel('val-num').value=(json["value"] == 0) ? '' : json["value"];
        let el = gel('period-num');
        el.value = json["period_id"];
        doChangeNumPeriod(el);
        validate_num();
    }else{
        gel('val-day').value=json["value"];
        gel('days').value = json["day_id"];
        let el = gel('period-day');
        el.value = json["period_id"];
        doChangeDayPeriod(el);
    }

    gel('time').value = json["time"];
    gel('date').value = json["date"];

    const params = json["params"]
    for(let i=0; i < params.length; i++ ){
        addParam('new-params', false, params[i]["name"], params[i]["value"]);
    }

    gel('note').focus();
    window.scrollTo(0, document.body.scrollHeight);
}

function GetTaskParams(task_id){
    const url = "/verify/schedule/task/params/get/"+task_id
    JsonGetMethod(url, success_task_params);
}

function success_task_params(json){
    for(let i=0; i < json.length; i++ ){
        addParam('new-params', true, json[i]["name"]);
    }
}

function doChangeTask(){
    let el = gel('task');
    let select_id = el.options[el.selectedIndex].value;
    clearParams();
    GetTaskParams(select_id);
}

function runJob(job_id){
    const url = "/verify/schedule/job/run/"+job_id
    showWaitWin('popup-wait');
    JsonPutMethod(url, null, success_run);
}

function success_run(json){
    hideWaitWin('popup-wait');
}

function doChangeNumPeriod(el)
{
    if(!el){
        return
    }

    let num_val = gel('num-val');
    let day_val = gel('day-val');
    select_id = el.options[el.selectedIndex].value;
    if(select_id == 'day_of_week'){
        day_val.style.display = 'block';
        num_val.style.display = 'none';
        let daysel = gel('period-day');
        daysel.selectedIndex = el.selectedIndex;
        doDayChange();

    }else{
        day_val.style.display = 'none';
        num_val.style.display = 'block';
        doPeriodChange();
    }
}

function doChangeDayPeriod(el)
{
    if(!el){
        return
    }

    let num_val = gel('num-val');
    let day_val = gel('day-val');
    select_id = el.options[el.selectedIndex].value;
    if(select_id == 'day_of_week'){
        day_val.style.display = 'block';
        num_val.style.display = 'none';
        doDayChange();
    }else{
        day_val.style.display = 'none';
        num_val.style.display = 'block';       
        let numsel = gel('period-num');
        numsel.selectedIndex = el.selectedIndex;
        doPeriodChange();
    }
}

function validate(evt) {
    var theEvent = evt || window.event;
  
    // Handle paste
    if (theEvent.type === 'paste') {
        key = event.clipboardData.getData('text/plain');
    } else {
    // Handle key press
        var key = theEvent.keyCode || theEvent.which;
        key = String.fromCharCode(key);
    }
    var regex = /[0-9]|\./;
    if( !regex.test(key) ) {
      theEvent.returnValue = false;
      if(theEvent.preventDefault) theEvent.preventDefault();
    }
}

function SetSelectPeriod(period){
    sel = gel('period-num');
    for(let i=0; i < sel.length; i++){
        let key = sel[i].value;
        sel[i].innerHTML = period.get(key);
    }
}

function validate_num() {
    let val = gel('val-num').value;
    doPeriodChange();
    if (val > 20) {
        val = val % 10;
    }
    if(val.length == 0 || val == 1){
        SetSelectPeriod(Period2);
    }else if(val > 1 && val < 5){
        SetSelectPeriod(Period3);
    }else{
        SetSelectPeriod(Period1);
    }
}

function doPeriodChange(){
    let el = gel('period-num'); 
    let id = el.options[el.selectedIndex].value;
    gel('val-num').disabled = false;
    let val = gel('val-num').value
    switch (id){
        case 'sec':
        case 'min':
        case 'week':
            if (val == "" || val == 1 || (val > 20 && val % 10 == 1)){
                gel('lb0').innerHTML = 'Повторять каждую';
            }else{
                gel('lb0').innerHTML = 'Повторять каждые';
            }
            break;
        case 'hour':
        case 'day':
        case 'month':
            if (val == "" || val == 1 || (val > 20 && val % 10 == 1)){
                gel('lb0').innerHTML = 'Повторять каждый';
            }else{
                gel('lb0').innerHTML = 'Повторять каждые';
            }
            break;
        case 'one':
            gel('lb0').innerHTML = 'Выполнить';
            gel('val-num').disabled = true;
            break;
        default:
            alert("Unknown id: "+id);
    }
}

function doDayChange(){
    let el = gel('days');  
    let id = el.options[el.selectedIndex].value;
    switch (id) {
        case '1':
        case '2':
        case '4':
            gel('lb1').innerHTML = 'Повторять каждый';
            break;
        case '3':
        case '5':
        case '6':
            gel('lb1').innerHTML = 'Повторять каждую';
            break;
        case '7':
            gel('lb1').innerHTML = 'Повторять каждое';
            break;
    }
}

function addParam(parent_id, visible, name, value){
    let el = gel(parent_id);
    countParams = countParams +1;
    let id = countParams.toString();
    name = (name == null) ? "" : name;
    value = (value == null) ? "" : value;
    const node = 
    `<div class="field" id="param_` + id +`">
        <label class="label-jobs label-left" for="param-name">Параметр:</label>
        <input id="param_name_`+id+`" class="fld-input param-name" placeholder="Имя" value="`+name+`">
        <input id="param_value_`+id+`" class="fld-input param-value" placeholder="Значение" value="`+value+`">
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

function clearParams(){
    for(let i = countParams; i >= 1 ; i--) {
        let id = "param_"+i.toString();   
        gel(id).remove();
    }
    gel('add-param').innerHTML = ">>";
    countParams = 0;
}

function onActivate(job_id, active) {
    const url = "/verify/schedule/job/activate/"+job_id
    JsonPutMethod(url, null, success_reload);
}

