// save some bytes
function gel(e) {
    return document.getElementById(e);
}
const wifi_div = gel("wifi");
const connect_div = gel("connect");
const connect_manual_div = gel("connect_manual");
const connect_wait_div = gel("connect-wait");
const connect_details_div = gel("connect-details");
const personal_div = gel("personal");
const enterprise_div = gel("enterprise");
const httpd_div = gel("httpd");
const ipv4_set_div = gel("ipv4-settings");

const WIFI = 10;
const MANUAL = 11;

const PERSONAL = 20;
const ENTERPRISE = 21;
const IPV4 = 22;
const HTTP = 23;

var connType;
var selectedForm;

function docReady(fn) {
  // see if DOM is already available
  if (
      document.readyState === "complete" ||
      document.readyState === "interactive"
      ) {
          // call on next available tick
          setTimeout(fn, 1);
  } else {
      document.addEventListener("DOMContentLoaded", fn);
  }
}

var selectedSSID = "";
var refreshAPInterval = null;
var checkStatusInterval = null;

function stopCheckStatusInterval() {
  if (checkStatusInterval != null) {
      clearInterval(checkStatusInterval);
      checkStatusInterval = null;
  }
}

function stopRefreshAPInterval() {
  if (refreshAPInterval != null) {
      clearInterval(refreshAPInterval);
      refreshAPInterval = null;
  }
}

function startCheckStatusInterval() {
  checkStatusInterval = setInterval(checkStatus, 950);
}

function startRefreshAPInterval() {
  refreshAPInterval = setInterval(refreshAP, 3800);
}

docReady(async function () {
  gel("wifi-status").addEventListener(
      "click",
      () => {
      wifi_div.style.display = "none";
      document.getElementById("connect-details").style.display = "block";
      },
      false
  );

  gel("TogglePassword").addEventListener(
      "click",
      (e) => {
      const type = pwd.getAttribute('type') === 'password' ? 'text' : 'password';
      pwd.setAttribute('type', type);
      if(type == 'text'){
          gel("lpwd").setAttribute('class', 'toggle-password--off');
      } else{
          gel("lpwd").setAttribute('class', 'toggle-password--on');
      }
      },
      false
  );

  gel("TogglePEAPassword").addEventListener(
      "click",
      (e) => {
      const type = ppas.getAttribute('type') === 'password' ? 'text' : 'password';
      ppas.setAttribute('type', type);
      if(type == 'text'){
          gel("lppas").setAttribute('class', 'toggle-password--off');
      } else{
          gel("lppas").setAttribute('class', 'toggle-password--on');
      }
  },
      false
  );

  gel("ToggleTTLSPassword").addEventListener(
      "click",
      (e) => {
      const type = tpas.getAttribute('type') === 'password' ? 'text' : 'password';
      tpas.setAttribute('type', type);
      if(type == 'text'){
          gel("ltpas").setAttribute('class', 'toggle-password--off');
      } else{
          gel("ltpas").setAttribute('class', 'toggle-password--on');
      }
  },
      false
  );

  gel("ToggleHTTPassword").addEventListener(
      "click",
      (e) => {
      const type = hpas.getAttribute('type') === 'password' ? 'text' : 'password';
      hpas.setAttribute('type', type);
      },
      false
  );

  gel("manual_add").addEventListener(
      "click",
      (e) => {
      selectedSSID = e.target.innerText;

      connType = MANUAL;
      gel("ssid-pwd").textContent = selectedSSID;
      connect_manual_div.style.display = "block";
      gel("connect_btn").style.display = "block";
      wifi_div.style.display = "none";
      connect_div.style.display = "none";
      ipv4_set_div.style.display = "none";
      httpd_div.style.display = "none";
      gel("manual_ssid").value = "";
      setCB0Value('Personal');
      clearHTTPDForm();
      setCB4Value('DHCP');
      clearIPv4Form();

      gel("connect-success").display = "none";
      gel("http-client-success").display = "none";
      gel("connect-fail").display = "none";
      gel("http-client-fail").display = "none";
      },
      false
  );

  gel("wifi-list").addEventListener(
      "click",
      (e) => {
      selectedSSID = e.target.innerText;

      connType = WIFI;
      gel("ssid-pwd").textContent = selectedSSID;
      connect_div.style.display = "block";
      gel("connect_btn").style.display = "block";
      connect_manual_div.style.display = "none";
      wifi_div.style.display = "none";
      ipv4_set_div.style.display = "none";
      httpd_div.style.display = "none";
      setCB1Value('Personal');
      clearHTTPDForm();
      setCB4Value('DHCP');
      clearIPv4Form();
    // init_cancel();
      },
      false
  );

  gel("ok-details").addEventListener(
      "click",
      () => {
      connect_details_div.style.display = "none";
      wifi_div.style.display = "block";
      },
      false
  );

  gel("ok-credits").addEventListener(
      "click",
      () => {
      gel("credits").style.display = "none";
      gel("app").style.display = "block";
      },
      false
  );

  gel("acredits").addEventListener(
      "click",
      () => {
      event.preventDefault();
      gel("app").style.display = "none";
      gel("credits").style.display = "block";
      },
      false
  );

  gel("ok-connect").addEventListener(
      "click",
      () => {
      connect_wait_div.style.display = "none";
      wifi_div.style.display = "block";
      },
      false
  );

  gel("disconnect").addEventListener(
      "click",
      () => {
      gel("diag-disconnect").style.display = "block";
      gel("connect-details-wrap").classList.add("blur");
      },
      false
  );

  gel("no-disconnect").addEventListener(
      "click",
      () => {
      gel("diag-disconnect").style.display = "none";
      gel("connect-details-wrap").classList.remove("blur");
      },
      false
  );

  gel("yes-disconnect").addEventListener("click", async () => {
      stopCheckStatusInterval();
      selectedSSID = "";

      document.getElementById("diag-disconnect").style.display = "none";
      gel("connect-details-wrap").classList.remove("blur");

      await fetch("connect.json", {
      method: "DELETE",
      headers: {
          "Content-Type": "application/json",
      },
      body: { timestamp: Date.now() },
      });

      startCheckStatusInterval();

      connect_details_div.style.display = "none";
      wifi_div.style.display = "block";
  });

  //first time the page loads: attempt get the connection status and start the wifi scan
  await refreshAP();
  startCheckStatusInterval();
  startRefreshAPInterval();
});

function rssiToIcon(rssi) {
    if (rssi >= -60) {
      return "w0";
    } else if (rssi >= -67) {
      return "w1";
    } else if (rssi >= -75) {
      return "w2";
    } else {
      return "w3";
    }
}

async function refreshAP(url = "ap.json") {
    try {
      var res = await fetch(url);
      var access_points = await res.json();
      if (access_points.length > 0) {
            //sort by signal strength
            access_points.sort((a, b) => {
              var x = a["rssi"];
              var y = b["rssi"];
              return x < y ? 1 : x > y ? -1 : 0;
            });
        refreshAPHTML(access_points);
  }
    } catch (e) {
      console.info("Access points returned empty from /ap.json!");
    }
}

function refreshAPHTML(data) {
    var h = "";
    data.forEach(function (e, idx, array) {
      let ap_class = idx === array.length - 1 ? "" : " brdb";
      let rssicon = rssiToIcon(e.rssi);
      let auth = e.auth == 0 ? "" : "pw";
      h += `<div class="ape${ap_class}"><div class="${rssicon}"><div class="${auth}">${e.ssid}</div></div></div>\n`;
    });

    gel("wifi-list").innerHTML = h;
}

async function checkStatus(url = "status.json") {
    try {
      var response = await fetch(url);
      var data = await response.json();
      if (data && data.hasOwnProperty("ssid") && data["ssid"] != "") {
            if (data["ssid"] === selectedSSID) {
              // Attempting connection
              switch (data["urc"]) {
                    case 0:
                      console.info("Got connection!");
                      document.querySelector("#connected-to div div div span").textContent = data["ssid"];
                      document.querySelector("#connect-details h1").textContent =  data["ssid"];
                      gel("ip").textContent = data["ip"];
                      gel("netmask").textContent = data["netmask"];
                      gel("gw").textContent = data["gw"];
                      gel("wifi-status").style.display = "block";

                      //unlock the wait screen if needed
                      gel("ok-connect").disabled = false;

                      //update wait screen
                      gel("loading").style.display = "none";
                      gel("connect-success").style.display = "block";
                      gel("connect-fail").style.display = "none";
                      break;
                    case 1:
                      console.info("Connection attempt failed!");
                      document.querySelector("#connected-to div div div span").textContent = data["ssid"];
                      document.querySelector("#connect-details h1").textContent = data["ssid"];
                      gel("ip").textContent = "0.0.0.0";
                      gel("netmask").textContent = "0.0.0.0";
                      gel("gw").textContent = "0.0.0.0";

                      //don't show any connection
                      gel("wifi-status").display = "none";

                      //unlock the wait screen
                      gel("ok-connect").disabled = false;

                      //update wait screen
                      gel("loading").display = "none";
                      gel("connect-fail").style.display = "block";
                      gel("connect-success").style.display = "none";
                      break;
              }
            } else if (data.hasOwnProperty("urc") && data["urc"] === 0) {
              console.info("Connection established");
              //ESP32 is already connected to a wifi without having the user do anything
              if (gel("wifi-status").style.display == "" || gel("wifi-status").style.display == "none" ) {
                    document.querySelector("#connected-to div div div span").textContent = data["ssid"];
                    document.querySelector("#connect-details h1").textContent = data["ssid"];
                    gel("ip").textContent = data["ip"];
                    gel("netmask").textContent = data["netmask"];
                    gel("gw").textContent = data["gw"];
                    gel("wifi-status").style.display = "block";
              }
            }		
            if(data.hasOwnProperty("httpc") && data["httpc"] === 1){
              gel("http-client-success").style.display = "block";
              gel("http-client-fail").style.display = "none";
            }	else{
              gel("http-client-success").style.display = "none";
              gel("http-client-fail").style.display = "block";
            }
      } else if (data.hasOwnProperty("urc") && data["urc"] === 2) {
            console.log("Manual disconnect requested...");
            if (gel("wifi-status").style.display == "block") {
              gel("wifi-status").style.display = "none";
            }
      }
    } catch (e) {
      console.info("Was not able to fetch /status.json");
    }
}

async function performConnect(conntype) {
    //stop the status refresh. This prevents a race condition where a status
    //request would be refreshed with wrong ip info from a previous connection
    //and the request would automatically shows as succesful.
    stopCheckStatusInterval();

  //stop refreshing wifi list
    stopRefreshAPInterval();

    var pwd, security, authentication, identity, cert_files, ca_file_size;
    var peap_user, peap_passwd, ttls_auth, ttls_user, ttls_passwd;
    var tls_crt_file_size, tls_key_file_size;
    var address, port, httpd_ca_size, httpd_client_crt_size, httpd_client_key_size, httpd_user, httpd_passw;
    var ipv4_addr, ipv4_mask, ipv4_gate, ipv4_dns, ipv4_ntp, ipv4_timezone;
    if (conntype == MANUAL) {
      //Grab the manual SSID and PWD
      selectedSSID = gel("manual_ssid").value;
    } 
    pwd = gel("pwd").value;
    security = gel("cb1").value;
    authentication = gel("cb2").value;
    identity = gel("ide").value;
    cert_files = new FormData();
    if(gel("check2").checked == false){
          ca_file_size = 0;
    }else{
          ca_file_size = gel("cas").files[0].size;
          cert_files.append('ca_file', gel("cas").files[0]);
    }
    peap_user = gel("pusr").value;
    peap_passwd = gel("ppas").value;
    ttls_auth = gel("cb3").value;
    ttls_user = gel("tusr").value;
    ttls_passwd = gel("tpas").value;
    if(gel("crt").value.length == 0){
          tls_crt_file_size = 0;
    }else{
          tls_crt_file_size = gel("crt").files[0].size;
          cert_files.append('crt_file', gel("crt").files[0]);
    }
    if(gel("key").value.length == 0){
          tls_key_file_size = 0;
          tls_key_paswd = "";
    }else{
          tls_key_file_size = gel("key").files[0].size;
          cert_files.append('key_file', gel("key").files[0]);
    }
    address = gel("addr").value;
    port = gel("port").value;
    if(gel("cah").value.length == 0){
          httpd_ca_size = 0;
    }else{
          httpd_ca_size = gel("cah").files[0].size;
          cert_files.append('httpd_ca', gel("cah").files[0]);
    }
    if(gel("client-crt").value.length == 0){
          httpd_client_crt_size = 0;
    }else{
      httpd_client_crt_size = gel("client-crt").files[0].size;
          cert_files.append('httpd_crt', gel("client-crt").files[0]);
    }
    if(gel("client-key").value.length == 0){
          httpd_client_key_size = 0;
    }else{
      httpd_client_key_size = gel("client-key").files[0].size;
          cert_files.append('httpd_key', gel("client-key").files[0]);
    }
    httpd_user = gel("husr").value;
    httpd_passw = gel("hpas").value;

    ipv4_addr = gel("ip_addr").value;
    ipv4_mask = gel("mask").value;
    ipv4_gate = gel("gate").value;
    ipv4_dns = gel("dns").value;
    ipv4_ntp = gel("ntp").value;
    ipv4_timezone = gel("cb-tz").value;

    //reset connection
    gel("loading").style.display = "block";
    gel("connect-success").style.display = "none";
    gel("http-client-success").style.display = "none";
    gel("connect-fail").style.display = "none";
    gel("http-client-fail").style.display = "none";

  gel("ok-connect").disabled = true;
  gel("ssid-wait").textContent = selectedSSID;
  connect_div.style.display = "none";
  connect_manual_div.style.display = "none";
  httpd_div.style.display = "none";
  ipv4_set_div.style.display = "none";
  connect_wait_div.style.display = "block";
  
  await fetch("connect.json", {
      method: "POST",
      headers: {
            "Content-Type": "multipart/form-data; boundary=--WebKitFormBoundaryfgtsKTYLST7PNUVD",
            "X-Custom-ssid": selectedSSID,
            "X-Custom-pwd": pwd,
            "X-Custom-security": security,
            "X-Custom-authentication": authentication,
            "X-Custom-identity": identity,
            "X-Custom-ca-file-size": ca_file_size,
            "X-Custom-peap-user": peap_user,
            "X-Custom-peap-passwd": peap_passwd,
            "X-Custom-ttls-auth": ttls_auth,
            "X-Custom-ttls-user": ttls_user,
            "X-Custom-ttls-passwd": ttls_passwd,
            "X-Custom-tls-crt-file-size": tls_crt_file_size,
            "X-Custom-tls-key-file-size": tls_key_file_size,
            "X-Custom-httpd-address": address,
            "X-Custom-httpd-port": port,
            "X-Custom-httpd-ca-size": httpd_ca_size,
            "X-Custom-httpd-crt-size": httpd_client_crt_size,
            "X-Custom-httpd-key-size": httpd_client_key_size,
            "X-Custom-httpd-username": httpd_user,
            "X-Custom-httpd-password": httpd_passw,
            "X-Custom-ipv4-addr": ipv4_addr,
            "X-Custom-ipv4-mask": ipv4_mask,
            "X-Custom-ipv4-gate": ipv4_gate,
            "X-Custom-ipv4-dns": ipv4_dns,
            "X-Custom-ipv4-ntp": ipv4_ntp,
            "X-Custom-ipv4-timezone": ipv4_timezone, 
      },
          body: cert_files
    });

    //now we can re-set the intervals regardless of result
    startCheckStatusInterval();
    startRefreshAPInterval();
}

function doCB1Change(value) {
    if (value === 'Personal') {
      enterpriseHide();
      personalShow();
      //setCB2Value('NONE');
      checkPasswordLength("pwd");
      selectedForm = PERSONAL;
    }
    else if (value === 'Enterprise') {
      enterpriseShow();
      personalHile();
      gel("ide").value = "";
      setCB2Value('PEAP');
      selectedForm = ENTERPRISE;
    }
}

function doCB2Change(value) {
  gel("check2").checked = false;
  doCheck2(false);
  gel("cas").value = "";
  switch(value){
        case 'TLS':
          gel("tls_sec").style.display = "block";
          gel("peap_sec").style.display = "none";
          gel("ttls_sec").style.display = "none";
          gel("crt").value = "";
          gel("key").value = "";
          doCRTFileSelect(true);
          break;
        case 'TTLS':
          gel("ttls_sec").style.display = "block";
          gel("tls_sec").style.display = "none";
          gel("peap_sec").style.display = "none";
          gel("tusr").value = "";
          gel("tpas").value = "";
          break;
        case 'PEAP':
          gel("peap_sec").style.display = "block";
          gel("tls_sec").style.display = "none";
          gel("ttls_sec").style.display = "none";
          //gel("ide").value = "";
          gel("pusr").value = "";
          gel("ppas").value = "";
          //gel("join").disabled = false;
          break;
        default:
          gel("tls_sec").style.display = "none";
          gel("peap_sec").style.display = "none";
          gel("ttls_sec").style.display = "none";
  }
}

function selectElement(id, valueToSelect){
  let el = gel(id);
  el.value = valueToSelect;
}

function setCB4Value(value){
  gel("cb4").value = value;
  doCB4Change(value);
}

function setCB0Value(value) {
  gel("cb0").value = value;
  doCB1Change(value);
}

function setCB1Value(value) {
    gel("cb1").value = value;
    doCB1Change(value);
}

function setCB2Value(value) {
    gel("cb2").value = value;
    doCB2Change(value);
}

function checkPasswordLength(fld) {
    var value = gel(fld).value;
    disableJoinBtn(value.length < 8);
}

function doCheck2(checked) {
    gel("cas").disabled = !checked;  
    if(!checked) gel("cas").value="";
     checkWiFiFormFill();
}

function checkWiFiFormFill(manual = false) {
  disableJoinBtn(false);
  if(manual && gel("manual_ssid").value.length == 0){
    disableJoinBtn(true); 
  }else if(personal_div.style.display == "block") {
    checkPasswordLength("pwd");
  }else{      
    var ident = gel("ide").value.length;
    if(gel("peap_sec").style.display  == "block"){
      var user = gel("pusr").value.length;
      var pass = gel("ppas").value.length;
      if (user == 0 || pass == 0 || ident == 0) {
          disableJoinBtn(true); 
      } else {
          var ca = gel("check2").checked;
          if(ca == true){
                var ca_file = gel("cas").files.length;
                if(ca_file == 0) {
                    disableJoinBtn(true);
                }
          }
      }  
    }else if(gel("ttls_sec").style.display == "block"){
      var user = gel("tusr").value.length;
      var pass = gel("tpas").value.length;
      if (user == 0 || pass == 0 || ident == 0) {
          disableJoinBtn(true); 
      } else {
          var ca = gel("check2").checked;
          if(ca == true){
                var ca_file = gel("cas").files.length;
                if(ca_file == 0) {
                    disableJoinBtn(true);
                }
          }
      }  
    }else if(gel("tls_sec").style.display == "block"){
      var crt = gel("crt").value.length;
      var key = gel("key").value.length;
      if (crt == 0 || key == 0 || ident == 0){
          disableJoinBtn(true); 
      } else {
          var ca = gel("check2").checked;
          if(ca == true){
                var ca_file = gel("cas").files.length;
                if(ca_file == 0) {
                    disableJoinBtn(true);
                }
          }
      }  
    }
  }
}

function doCRTFileSelect(empty) {
    gel("key").disabled = empty;
    checkWiFiFormFill();
}

function onNextClick(){
  //alert(selectedForm);
  switch(selectedForm){
    case PERSONAL:
    case ENTERPRISE:
      gel("join").value = ">>";
      gel("cancel").value = "<<";
      connect_div.style.display = "none";
      connect_manual_div.style.display = "none";
      personal_div.style.display = "none";
      httpd_div.style.display = "none";
      ipv4_set_div.style.display = "block";
      checkIPv4FormFill();
      enterpriseHide();
      selectedForm = IPV4;
      break;
    case IPV4:
      gel("join").value = "Join";
      gel("cancel").value = "<<";
      connect_div.style.display = "none";
      connect_manual_div.style.display = "none";
      personal_div.style.display = "none";
      httpd_div.style.display = "block";
      ipv4_set_div.style.display = "none";
      enterpriseHide();
      checkHttpFormFill();
      selectedForm = HTTP;
      break;
    case HTTP:
      gel("join").value = ">>";
      gel("cancel").value = "Cancel";
      disableJoinBtn(false);
      gel("connect_btn").style.display = "none";
      performConnect(connType);
  }
}	  

function onPrevClick(){
  //alert(selectedForm);
  var join = gel("join").value;
  var cancel = gel("cancel").value;
  switch(selectedForm){
    case HTTP:
      httpd_div.style.display = "none";
      ipv4_set_div.style.display = "block";
      gel("join").value = ">>";
      gel("cancel").value  = "<<";
      selectedForm = IPV4;
      checkIPv4FormFill();
      break;
    case IPV4:
      ipv4_set_div.style.display = "none";
      gel("join").value = ">>";
      gel("cancel").value  = "Cancel";
      var cb0 = gel("cb0");
      var cb1 = gel("cb1");
      if(connType == MANUAL){
        connect_manual_div.style.display = "block";
        if(cb0.options[cb0.selectedIndex].value == "Personal"){
          personalShow();  
        }else{
          enterpriseShow();
        }
        checkWiFiFormFill(true);
      }else{
        connect_div.style.display = "block";
        if(cb1.options[cb1.selectedIndex].value == "Personal"){
          personalShow();  
        }else{
          enterpriseShow();
        }
        checkWiFiFormFill();
      }
      break;
    case PERSONAL:
    case ENTERPRISE:
      gel("connect_btn").style.display = "none";
      performCancel();
  }
}	  

function doCheck4(checked) {
  gel("husr").disabled = !checked;  
  gel("hpas").disabled = !checked;  
  if(!checked) {
    gel("husr").value="";
    gel("hpas").value="";
  }
  checkHttpFormFill();
}  

function performCancel() {
  selectedSSID = "";
  wifi_div.style.display = "block";
  connect_div.style.display = "none";
  connect_manual_div.style.display = "none";
  ipv4_set_div.style.display = "none";
  gel("manual_ssid").value = selectedSSID;
  gel("pwd").value = "";
  enterpriseHide();
  if(connType == MANUAL){
    setCB0Value('Personal');
  }else{
    setCB1Value('Personal');
  }
  personalHile();
}

function clearHTTPDForm(){
    gel("addr").value = "192.168.1.3";
    gel("port").value = "443";
    gel("cah").value = "";
    gel("cah").disabled = false;
    gel("client-crt").value = "";
    gel("client-crt").disabled = false;
    gel("client-key").value = "";
    gel("client-key").disabled = false;
    gel("husr").value = "";
    gel("hpas").value = "";
    gel("husr").disabled = true;
    gel("hpas").disabled = true;
    gel("check4").checked = false;
}

function disableJoinBtn(value)
{
    if(value === true) {
      gel("join").disabled = true;
      gel("join").style.color = "grey";
    } else {
      gel("join").disabled = false;
      gel("join").style.color = "white";
    }
}

function checkHttpFormFill(){
    disableJoinBtn(false);
    /*var _addr = gel("addr").value.length;
    var _port = gel("port").value.length;
    if(_addr == 0 || _port == 0 || !validIP_OR_NS(gel("addr"))){
      disableJoinBtn(true);   
    }else{
      if(gel("cah").value.length == 0){
            disableJoinBtn(true);
      }
      var login = gel("check4").checked;
      if(login == true){
          var user = gel("husr").value.length;
          var pass = gel("hpas").value.length;
          if(user == 0 || pass == 0){
                disableJoinBtn(true);
          }
      }
    }*/
}	

function enterpriseHide(){
  enterprise_div.style.display = "none";	
  gel("peap_sec").style.display = "none";	
  gel("ttls_sec").style.display = "none";	
  gel("tls_sec").style.display = "none";	
}

function enterpriseShow(){
  selectedForm = ENTERPRISE;
  enterprise_div.style.display = "block";
  var cb2 = gel("cb2");
  switch(cb2.options[cb2.selectedIndex].value){
      case "PEAP":
          gel("peap_sec").style.display = "block";
          break;
      case "TLS":
          gel("tls_sec").style.display = "block";	
          break;
      case "TTLS":
          gel("ttls_sec").style.display = "block";	
          break;
  }
}

function personalHile(){
  personal_div.style.display = "none";
}

function personalShow(){
  selectedForm = PERSONAL;
  personal_div.style.display = "block";
}

function doCB4Change(value) {
  switch(value){
        case 'DHCP':
          gel("ip_addr").disabled = true;
          gel("mask").disabled = true;
          gel("gate").disabled = true;
          gel("ip_addr").value = "";
          gel("mask").value = "";
          gel("gate").value = "";
          checkIPv4FormFill();
          break;
        case 'Manual':
          gel("ip_addr").disabled = false;
          gel("mask").disabled = false;
          gel("gate").disabled = false;
          checkIPv4FormFill();
          break;
  }
}

function clearIPv4Form(){
  gel("ip_addr").value = "";
  gel("mask").value = "";
  gel("gate").value = "";
  gel("dns").value = "";
  gel("ntp").value = "by.pool.ntp.org";
  doCB4Change('DHCP');
}

function checkIPv4FormFill(){
  disableJoinBtn(false);
  if(gel("ip_addr").disabled === false && !validIPv4Address(gel("ip_addr")))  
    disableJoinBtn(true);
  if(gel("mask").disabled === false && !validIPv4Address(gel("mask")))  
    disableJoinBtn(true);
  if(gel("gate").disabled === false && !validIPv4Address(gel("gate")))  
    disableJoinBtn(true);
  if(gel("dns").disabled === false && !validIPv4_OR_Empty(gel("dns")))  
    disableJoinBtn(true);
  if(gel("ntp").disabled === false && !validIPv4_OR_NS_OR_Empty(gel("ntp")))
    disableJoinBtn(true);
}

function validIPv4_OR_Empty(data){
  return validIPv4Address(data) || data.value.length == 0;
}

function validIPv4_OR_NS_OR_Empty(data){
  return validDNSName(data) || validIPv4Address(data) || data.value.length == 0;
}

function validIP_OR_NS(data){
  return validDNSName(data) || validIPv4Address(data);
}

function validIPv4Address(ipaddress) {
  var regEx = /^(25[0-5]|2[0-4][0-9]|[01]?[0-9][0-9]?)\.(25[0-5]|2[0-4][0-9]|[01]?[0-9][0-9]?)\.(25[0-5]|2[0-4][0-9]|[01]?[0-9][0-9]?)\.(25[0-5]|2[0-4][0-9]|[01]?[0-9][0-9]?)$/;
  if(ipaddress.value.match(regEx)){
    return true;
  }else {
    return false;
  }  
}

function validDNSName(name){
  var regEx = /^((([a-zA-Z]{1,2})|([0-9]{1,2})|([a-zA-Z0-9]{1,2})|([a-zA-Z0-9][a-zA-Z0-9-]{1,61}[a-zA-Z0-9]))\.)+[a-zA-Z]{2,6}$/;
  if(name.value.match(regEx)){
    return true;
  }else {
    return false;
  }  
}

function ValidateIPaddress(_obj)
{
    var ipformat = /^(25[0-5]|2[0-4][0-9]|[01]?[0-9][0-9]?)\.(25[0-5]|2[0-4][0-9]|[01]?[0-9][0-9]?)\.(25[0-5]|2[0-4][0-9]|[01]?[0-9][0-9]?)\.(25[0-5]|2[0-4][0-9]|[01]?[0-9][0-9]?)$/;
    var nsformat = /^((([a-zA-Z]{1,2})|([0-9]{1,2})|([a-zA-Z0-9]{1,2})|([a-zA-Z0-9][a-zA-Z0-9-]{1,61}[a-zA-Z0-9]))\.)+[a-zA-Z]{2,6}$/;
    var ipaddr = gel("ip_addr");
    var gateway = gel("gate");
    var subnet = gel("mask");
    var dns = gel("dns");
    var ntp = gel("ntp");
    var counter = 0;

    if(ipaddr.value.match(ipformat) || ipaddr.value.length === 0) {
      _obj.focus();
    } else {
        alert("You have entered an invalid IP Address!");
        ipaddr.focus();
        return (false);
    }
    if(gateway.value.match(ipformat) || gateway.value.length === 0) {
      _obj.focus();
    } else {
        window.alert("You have entered an invalid GATEWAY Address!");
        gateway.focus();
        return (false);
    }            
    if(subnet.value.match(ipformat) || subnet.value.length === 0) {
      _obj.focus();
    } else {
        window.alert("You have entered an invalid SUBNET Address!");
        subnet.focus();
        return (false);
    }            
    if(dns.value.match(ipformat) || dns.value.length === 0) {
      _obj.focus();
    } else {
        window.alert("You have entered an invalid DNS Address!");
        dns.focus();
        return (false);
    }
    if(ntp.value.length === 0 || ntp.value.match(nsformat) || ntp.value.match(ipformat)) {
      _obj.focus();
    } else {
      window.alert("You have entered an invalid NTP Server!");
      ntp.focus();
      return (false);
    }
}
