function createBox(world, x, y, width, height, fixed) {
    var box = new b2BoxDef();
    if (!fixed) box.density = 1.0;
    box.extents.Set(width, height);
    var boxBd = new b2BodyDef();
    boxBd.AddShape(box);
    boxBd.position.Set(x,y);
    return world.CreateBody(boxBd)
}

