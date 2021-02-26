//server.js

var express     = require('express');
var app         = express();
var mongoose    = require('mongoose'),
		Schema = mongoose.Schema;


var pickersSchema = new Schema({
	pickerId: 	{ type: String },
	action: 	{ type: String },
    status: 	{ type: String },
});

// Conexión con la base de datos
mongoose.connect('mongodb://localhost:27017/sa3ir');

// Configuración
app.configure(function() {
    // Localización de los ficheros
    app.use(express.static(__dirname + '/public'));
    // Muestra un log de todos los request en la consola        
    app.use(express.logger('dev')); 
    // Permite cambiar el HTML con el método POST                   
    app.use(express.bodyParser());
    // DELETE y PUT                      
    app.use(express.methodOverride());                  
});



var pickers = mongoose.model('Pickers', pickersSchema);


// Implementación de la API REST

app.get('/api/pickers', function(req, res) {              
    pickers.find(function(err, todos) {
        if(err) {
            res.send(err);
        }
        res.json(todos);
    });
});


var updateDB = function(req, res, id, action, status) {
	pickers.findOne(
		{pickerId: id}, 
		function(err, picker) {
			if(!err && picker!=null) {
				picker.pickerId   = id;
	  			picker.action    = action;
                picker.status = status;
                picker.save(
					function(err) {
			  			if(!err) {
			  				console.log('Updated');
			  			} else {
			  				console.log('ERROR: ' + err);
			  			}
						pickers.find(function(err, todos) {
						    if(err){
                                res.send(err);
						    }
						    res.json(todos);
						});
		  				//res.send(picker);
		  			}
				);
			}
			else {
                pickers.create({
					pickerId: id,
					action: action,
					status: status,
			 	   }, function(err, picker){
					if(err) {
					    res.send(err);
					}

					pickers.find(function(err, results) {
					    if(err){
                            res.send(err);
                        }
					    res.json(results);
					});
				    });
			}//if
		}//function
	);
}


app.post('/api/picker1_request', function(req, res) {   
	console.log('picker1_request');
	updateDB(req, res, "1", "request_trolley", "Waiting for request trolley...");
});

app.post('/api/picker1_deliver', function(req, res) { 
	console.log('picker1_deliver'); 
	updateDB(req, res, "1", "deliver_trolley", "Waiting for deliver trolley..."); 
  	
});

app.post('/api/picker1_deliver_full', function(req, res) {   
console.log('picker1_deliver_full'); 
	updateDB(req, res, "1", "deliver_trolley_full", "Waiting for deliver full trolley");
});

app.post('/api/picker2_request', function(req, res) {   
	console.log('picker2_request');
	updateDB(req, res, "2", "request_trolley", "Waiting for request trolley...");
});

app.post('/api/picker2_deliver', function(req, res) { 
	console.log('picker2_deliver'); 
	updateDB(req, res, "2", "deliver_trolley", "Waiting for deliver trolley"); 
  	
});

app.post('/api/picker2_deliver_full', function(req, res) {   
console.log('picker2_deliver_full'); 
	updateDB(req, res, "2", "deliver_trolley_full", "Waiting for deliver full trolley");
});


app.post('/api/picker3_request', function(req, res) {   
	console.log('picker3_request');
	updateDB(req, res, "3", "request_trolley", "Waiting for request trolley...");
});

app.post('/api/picker3_deliver', function(req, res) { 
	console.log('picker3_deliver'); 
	updateDB(req, res, "3", "deliver_trolley", "Waiting for deliver trolley"); 
  	
});

app.post('/api/picker3_deliver_full', function(req, res) {   
console.log('picker3_deliver_full'); 
	updateDB(req, res, "3", "deliver_trolley_full", "Waiting for deliver full trolley");
});

app.post('/api/picker4_request', function(req, res) {   
	console.log('picker4_request');
	updateDB(req, res, "4", "request_trolley", "Waiting for request trolley...");
});

app.post('/api/picker4_deliver', function(req, res) { 
	console.log('picker4_deliver'); 
	updateDB(req, res, "4", "deliver_trolley", "Waiting for deliver trolley"); 
  	
});

app.post('/api/picker4_deliver_full', function(req, res) {   
console.log('picker4_deliver_full'); 
	updateDB(req, res, "4", "deliver_trolley_full", "Waiting for deliver full trolley");
});

app.post('/api/picker1_remove_action', function(req, res) {   
console.log('picker1_remove_action'); 
	updateDB(req, res, "1", "", "Request processed. Please wait.");
});

app.post('/api/picker2_remove_action', function(req, res) {   
console.log('picker2_remove_action'); 
	updateDB(req, res, "2", "", "Request processed. Please wait.");
});

app.post('/api/picker3_remove_action', function(req, res) {   
console.log('picker3_remove_action'); 
	updateDB(req, res, "3", "", "Request processed. Please wait.");
});

app.post('/api/picker4_remove_action', function(req, res) {   
console.log('picker4_remove_action'); 
	updateDB(req, res, "4", "", "Request processed. Please wait.");
});

app.post('/api/picker1_has_trolley', function(req, res) {   
console.log('picker1_has_trolley'); 
	updateDB(req, res, "1", "", "The trolley is here");
});

app.post('/api/picker1_deliver_trolley', function(req, res) {   
console.log('picker1_deliver_trolley'); 
	updateDB(req, res, "1", "", "The robot has picked up the trolley. Thanks!");
});

app.post('/api/picker2_has_trolley', function(req, res) {   
console.log('picker2_has_trolley'); 
	updateDB(req, res, "2", "", "The trolley is here");
});

app.post('/api/picker2_deliver_trolley', function(req, res) {   
console.log('picker2_deliver_trolley'); 
	updateDB(req, res, "2", "", "The robot has picked up the trolley. Thanks!");
});

app.post('/api/picker3_has_trolley', function(req, res) {   
console.log('picker3_has_trolley'); 
	updateDB(req, res, "3", "", "The trolley is here");
});

app.post('/api/picker3_deliver_trolley', function(req, res) {   
console.log('picker3_deliver_trolley'); 
	updateDB(req, res, "3", "", "The robot has picked up the trolley. Thanks!");
});

app.post('/api/picker4_has_trolley', function(req, res) {   
console.log('picker4_has_trolley'); 
	updateDB(req, res, "4", "", "The trolley is here");
});

app.post('/api/picker4_deliver_trolley', function(req, res) {   
console.log('picker4_deliver_trolley'); 
	updateDB(req, res, "4", "", "The robot has picked up the trolley. Thanks!");
});


//ERROR methods:

app.post('/api/picker1_navigation_error', function(req, res) {   
console.log('picker1_navigation_error'); 
	updateDB(req, res, "1", "", "ERROR: The robot is not able to reach the goal");
});

app.post('/api/picker1_deliver_error', function(req, res) {   
console.log('picker1_deliver_error'); 
	updateDB(req, res, "1", "", "ERROR: The robot is not able to deliver the trolley");
});

app.post('/api/picker1_object_error', function(req, res) {   
console.log('picker1_object_error'); 
	updateDB(req, res, "1", "", "ERROR: The robot did not find the trolley. Trying again...");
});

app.post('/api/picker1_pickup_error', function(req, res) {   
console.log('picker1_pickup_error'); 
	updateDB(req, res, "1", "", "ERROR: The robot is not able to pickup the trolley");
});

app.post('/api/picker2_navigation_error', function(req, res) {   
console.log('picker2_navigation_error'); 
	updateDB(req, res, "2", "", "ERROR: The robot is not able to reach the goal");
});

app.post('/api/picker2_deliver_error', function(req, res) {   
console.log('picker2_deliver_error'); 
	updateDB(req, res, "2", "", "ERROR: The robot is not able to deliver the trolley");
});

app.post('/api/picker2_object_error', function(req, res) {   
console.log('picker2_object_error'); 
	updateDB(req, res, "2", "", "ERROR: The robot did not find the trolley. Trying again...");
});

app.post('/api/picker2_pickup_error', function(req, res) {   
console.log('picker2_pickup_error'); 
	updateDB(req, res, "2", "", "ERROR: The robot is not able to pickup the trolley");
});

app.post('/api/picker3_navigation_error', function(req, res) {   
console.log('picker3_navigation_error'); 
	updateDB(req, res, "3", "", "ERROR: The robot is not able to reach the goal");
});

app.post('/api/picker3_deliver_error', function(req, res) {   
console.log('picker3_deliver_error'); 
	updateDB(req, res, "3", "", "ERROR: The robot is not able to deliver the trolley");
});

app.post('/api/picker3_object_error', function(req, res) {   
console.log('picker3_object_error'); 
	updateDB(req, res, "3", "", "ERROR: The robot did not find the trolley. Trying again...");
});

app.post('/api/picker3_pickup_error', function(req, res) {   
console.log('picker3_pickup_error'); 
	updateDB(req, res, "3", "", "ERROR: The robot is not able to pickup the trolley");
});

app.post('/api/picker4_navigation_error', function(req, res) {   
console.log('picker4_navigation_error'); 
	updateDB(req, res, "4", "", "ERROR: The robot is not able to reach the goal");
});

app.post('/api/picker4_deliver_error', function(req, res) {   
console.log('picker4_deliver_error'); 
	updateDB(req, res, "4", "", "ERROR: The robot is not able to deliver the trolley");
});

app.post('/api/picker4_object_error', function(req, res) {   
console.log('picker4_object_error'); 
	updateDB(req, res, "4", "", "ERROR: The robot did not find the trolley. Trying again...");
});

app.post('/api/picker4_pickup_error', function(req, res) {   
console.log('picker4_pickup_error'); 
	updateDB(req, res, "4", "", "ERROR: The robot is not able to pickup the trolley");
});


// DELETE un picker específico y devuelve todos tras borrarlo.
app.delete('/api/pickers/:picker', function(req, res) {     
    pickers.remove({
        _id: req.params.todo
    }, function(err, todo) {
        if(err){
            res.send(err);
        }

        pickers.find(function(err, todos) {
            if(err){
                res.send(err);
            }
            res.json(todos);
        });

    })
});


// Devuelve la página web
app.get('*', function(req, res) {                       
    res.sendfile('./public/index.html');                
});


// Escuchando en el puerto 8080
app.listen(8080, function() {
    console.log('App listening on port 8080');
});
