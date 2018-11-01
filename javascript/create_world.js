function baseWorld(){
        
    // Basic initialization routines from the Box2dJS demos.
    var worldAABB = new b2AABB();
	worldAABB.minVertex.Set(-1000, -1000);
	worldAABB.maxVertex.Set(1000, 1000);
	var gravity = new b2Vec2(0, 300);
	var doSleep = true;
	var world = new b2World(worldAABB, gravity, doSleep);
	var groundSd = new b2BoxDef();
	groundSd.extents.Set(1000, 50);
	groundSd.restitution = 0.2;
	var groundBd = new b2BodyDef();
	groundBd.AddShape(groundSd);
	groundBd.position.Set(-500, 340);
        world.CreateBody(groundBd)
	createBox(world, 0, 125, 10, 250);
	createBox(world, 500, 125, 10, 250);
	return world;
}

function createWorld() {
    world = baseWorld();
    return world;
   
}
