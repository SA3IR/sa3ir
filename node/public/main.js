angular.module('angularSa3ir', []);
function mainController($scope, $http) {
    $scope.formData = {};

    $http.get('/api/pickers')
        .success(function(data) {
            $scope.todos = data;
        })
        .error(function(data) {
            console.log('Error: ' + data);
        });

    // Petición del picker id
    $scope.callTrolley = function(id){
        var path;	
        if(id == '10'){
            path = '/api/picker1_request';
            //document.getElementById('status_picker1').innerHTML = 'Request action processed.'; 
        }else if (id == '11'){
            path = '/api/picker1_deliver';
           // document.getElementById('status_picker1').innerHTML = 'Deliver action processed.'; 
        }else if (id == '12'){
            //document.getElementById('status_picker1').innerHTML = 'Deliver full action processed.'; 
           path = '/api/picker1_deliver_full';
        }
        else if(id == '20'){
            //document.getElementById('status_picker2').innerHTML = 'Request action processed.'; 
            path = '/api/picker2_request';
        }else if (id == '21'){
            //document.getElementById('status_picker2').innerHTML = 'Deliver action processed.'; 
            path = '/api/picker2_deliver';
        }else if (id == '22'){
            //document.getElementById('status_picker2').innerHTML = 'Deliver full action processed.'; 
            path = '/api/picker2_deliver_full';
        }
        else if(id == '30'){
            //document.getElementById('status_picker3').innerHTML = 'Request action processed.'; 
            path = '/api/picker3_request';
        }else if (id == '31'){
            //document.getElementById('status_picker3').innerHTML = 'Deliver action processed.'; 
            path = '/api/picker3_deliver';
        }else if (id == '32'){
           // document.getElementById('status_picker3').innerHTML = 'Deliver full action processed.'; 
            path = '/api/picker3_deliver_full';
        }
        else if(id == '40'){
            //document.getElementById('status_picker4').innerHTML = 'Request action processed.'; 
            path = '/api/picker4_request';
        }else if (id == '41'){
            //document.getElementById('status_picker4').innerHTML = 'Deliver action processed.'; 
            path = '/api/picker4_deliver';
        }else if (id == '42'){
            //document.getElementById('status_picker4').innerHTML = 'Deliver full action processed.'; 
            path = '/api/picker4_deliver_full';
        }
        console.log(path);
            $http.post(path, $scope.formData)
                .success(function(data) {
                    $scope.formData = {};
                    $scope.todos = data;
                    console.log(data);
                })
                .error(function(data) {
                    console.log('Error:' + data);
                });

	
    };

    $scope.deletePicker = function(id) {
        $http.delete('/api/pickers/' + id)
            .success(function(data) {
                $scope.todos = data;
                console.log(data);
            })
            .error(function(data) {
                console.log('Error:' + data);
            });
    };
   
   var temporizador = setInterval(
        function(){
            $http.get('/api/pickers')
                .success(function(data) {
                    $scope.todos = data;
                    console.log(data);
                    var jsonData = JSON.parse(JSON.stringify(data));
                    for (var i = 0; i < jsonData.length; i++) {
                        var picker = jsonData[i];
                       // console.log(picker.pickerId);
                        if(picker.pickerId == "1")
                        {
                            try {
                                document.getElementById('status_picker1').innerHTML = picker.status;
                            }catch(e) {/*console.error(e.message);*/}
                        }
                        else if (picker.pickerId == "2")
                        {
                            try{
                                document.getElementById('status_picker2').innerHTML = picker.status;
                            }
                            catch(e) {/*console.error(e.message);*/}
                        }
                        else if (picker.pickerId == "3")
                        {
                            try{
                                document.getElementById('status_picker3').innerHTML = picker.status;
                            }
                            catch(e) {/*console.error(e.message);*/}
                        }
                        else if (picker.pickerId == "4")
                        {
                            try{
                                document.getElementById('status_picker4').innerHTML = picker.status;
                            }
                            catch(e) {/*console.error(e.message);*/}
                        }
                    }
                })
                .error(function(data) {
                     document.getElementById('status_picker1').innerHTML = 'error'; 
                });
            
            
        },1000); // 10000ms = 10s
   
   
}
