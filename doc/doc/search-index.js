var searchIndex = JSON.parse('{\
"homing":{"doc":"","t":[3,3,8,3,3,8,3,3,3,3,11,11,12,12,11,11,11,11,11,11,11,11,11,11,11,11,11,11,11,11,11,11,11,11,11,11,11,11,11,11,11,11,11,11,11,12,12,10,11,11,11,11,11,11,11,11,11,11,11,11,11,11,11,11,11,11,11,11,11,11,11,11,11,12,12,12,11,11,11,11,11,11,11,11,11,11,11,11,11,5,10,11,11,11,11,11,11,12,11,11,11,11,11,12,12,12,12,12,11,11,11,11,11,11,11,11,11,11,11,11,11,11,11,11,11,11,11,11,11,11,11,11,11,11,11,11,11,11,11,11,12,12,12],"n":["Bee","Circle","Distance","Grid","Image","Obstacle","Segment","Vec2","VectorField","World","add","add_assign","avg_angular_error","bisector","borrow","borrow","borrow","borrow","borrow","borrow","borrow","borrow","borrow_mut","borrow_mut","borrow_mut","borrow_mut","borrow_mut","borrow_mut","borrow_mut","borrow_mut","clone","clone","clone","clone","clone","clone","clone","clone_into","clone_into","clone_into","clone_into","clone_into","clone_into","clone_into","collides","color","data","dist","dist","draw","eq","eq","eq","eq","eq","eq","eq","fmt","fmt","fmt","fmt","fmt","fmt","fmt","from","from","from","from","from","from","from","from","generate","grid","grid","height","home","index","index","into","into","into","into","into","into","into","into","into","len","main","map","map","merge","new","new","new","normalized","obstacles","partial_cmp","partial_cmp","partial_cmp","partial_cmp","partial_cmp","position","position","radius","segments","snapshot","sub","to_owned","to_owned","to_owned","to_owned","to_owned","to_owned","to_owned","try_from","try_from","try_from","try_from","try_from","try_from","try_from","try_from","try_into","try_into","try_into","try_into","try_into","try_into","try_into","try_into","type_id","type_id","type_id","type_id","type_id","type_id","type_id","type_id","vectors","width","width"],"q":["homing","","","","","","","","","","","","","","","","","","","","","","","","","","","","","","","","","","","","","","","","","","","","","","","","","","","","","","","","","","","","","","","","","","","","","","","","","","","","","","","","","","","","","","","","","","","","","","","","","","","","","","","","","","","","","","","","","","","","","","","","","","","","","","","","","","","","","","","","","","","","","",""],"d":["bee struct to hold information about the snapshot and its …","obstacle struct for circular objects","","Grid struct for all your grid needs origin of the Grid is …","datastructure to hold the Segments this will be used for …","trait for obstacles all obstacles will have to implement …","datastructure for Segments on the image circle","datastructure for 2d vectors","VectorField struct for storing all generated vectors","World that holds obstacles and the grid the bee is allowed …","","","","bisector for the Segment, radians ranges from 0..2Pi","","","","","","","","","","","","","","","","","","","","","","","","","","","","","","","checks if two segments collide/overlap returns true if the …","color of the Segment","data for the 2d vector","","","","","","","","","","","","","","","","","","Returns the argument unchanged.","Returns the argument unchanged.","Returns the argument unchanged.","Returns the argument unchanged.","Returns the argument unchanged.","Returns the argument unchanged.","Returns the argument unchanged.","Returns the argument unchanged.","","the grid that allows the bee to move","","height of the grid","","","","Calls <code>U::from(self)</code>.","Calls <code>U::from(self)</code>.","Calls <code>U::from(self)</code>.","","Calls <code>U::from(self)</code>.","Calls <code>U::from(self)</code>.","Calls <code>U::from(self)</code>.","Calls <code>U::from(self)</code>.","Calls <code>U::from(self)</code>.","","","maps the obstacle from a position to a Segment","","merges the check_with segment with the other given segments","","","","","list of obstacles in the world NOTE: the obstacles do not …","","","","","","position of the bee","center of the circle","radius of the circle","the segments that make up the image circle","snapshot of all obstacles","","","","","","","","","","","","","","","","","","","","","","","","","","","","","","","","","","width of the Segment, radians","width of the grid"],"i":[0,0,0,0,0,0,0,0,0,0,1,1,8,2,19,2,3,1,5,6,7,8,19,2,3,1,5,6,7,8,2,3,1,5,6,7,8,2,3,1,5,6,7,8,2,2,1,27,2,8,2,3,1,5,6,7,8,2,3,1,5,6,7,8,19,2,3,1,5,6,7,8,8,19,8,7,5,1,8,19,2,3,1,1,5,6,7,8,1,0,28,6,2,3,1,5,1,19,2,3,1,5,6,5,6,6,3,5,1,2,3,1,5,6,7,8,19,2,3,1,5,6,7,8,19,2,3,1,5,6,7,8,19,2,3,1,5,6,7,8,8,2,7],"f":[0,0,0,0,0,0,0,0,0,0,[[1,1]],[[1,1]],0,0,[[]],[[]],[[]],[[]],[[]],[[]],[[]],[[]],[[]],[[]],[[]],[[]],[[]],[[]],[[]],[[]],[2,2],[3,3],[[[1,[4]]],[[1,[4]]]],[5,5],[6,6],[7,7],[8,8],[[]],[[]],[[]],[[]],[[]],[[]],[[]],[[2,2],9],0,0,[[],10],[[2,2],10],[[8,11],[[14,[[13,[12]]]]]],[[2,2],9],[[3,3],9],[[[1,[15]],1],9],[[5,5],9],[[6,6],9],[[7,7],9],[[8,8],9],[[2,16],17],[[3,16],17],[[[1,[18]],16],17],[[5,16],17],[[6,16],17],[[7,16],17],[[8,16],17],[[]],[[]],[[]],[[]],[[]],[[]],[[]],[[]],[[5,19],8],0,0,0,[[5,19],[[1,[10]]]],[[1,20]],[[8,[1,[21]]],[[1,[20]]]],[[]],[[]],[[]],[[[1,[21]]],[[1,[10]]]],[[]],[[]],[[]],[[]],[[]],[1,10],[[]],[[[1,[21]]],[[22,[2]]]],[[6,[1,[21]]],[[22,[2]]]],[[2,[23,[2]]]],[[[1,[21]],23],3],[[],1],[[19,[1,[21]]],5],[1,[[1,[10]]]],0,[[2,2],[[22,[24]]]],[[3,3],[[22,[24]]]],[[[1,[25]],1],[[22,[24]]]],[[5,5],[[22,[24]]]],[[6,6],[[22,[24]]]],0,0,0,0,0,[[1,1]],[[]],[[]],[[]],[[]],[[]],[[]],[[]],[[],14],[[],14],[[],14],[[],14],[[],14],[[],14],[[],14],[[],14],[[],14],[[],14],[[],14],[[],14],[[],14],[[],14],[[],14],[[],14],[[],26],[[],26],[[],26],[[],26],[[],26],[[],26],[[],26],[[],26],0,0,0],"p":[[3,"Vec2"],[3,"Segment"],[3,"Image"],[8,"Clone"],[3,"Bee"],[3,"Circle"],[3,"Grid"],[3,"VectorField"],[15,"bool"],[15,"f32"],[15,"str"],[8,"Error"],[3,"Box"],[4,"Result"],[8,"PartialEq"],[3,"Formatter"],[6,"Result"],[8,"Debug"],[3,"World"],[15,"usize"],[15,"i32"],[4,"Option"],[3,"Vec"],[4,"Ordering"],[8,"PartialOrd"],[3,"TypeId"],[8,"Distance"],[8,"Obstacle"]]}\
}');
if (typeof window !== 'undefined' && window.initSearch) {window.initSearch(searchIndex)};
if (typeof exports !== 'undefined') {exports.searchIndex = searchIndex};
