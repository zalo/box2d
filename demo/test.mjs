// Simple Node.js test for Box2D bindings
import('./box2d_bindings.js').then(async (Box2DFactory) => {
    console.log('Loading Box2D...');
    
    const Box2D = await Box2DFactory.default();
    console.log('Box2D loaded successfully!');
    
    // Create world
    const world = new Box2D.World(0, -10); // gravity pointing down
    console.log('World created, is valid:', world.isValid());
    
    // Create a static ground body
    const ground = new Box2D.Body(world, 0, -10, Box2D.BodyType.staticBody);
    Box2D.ShapeFactory.createBox(ground, 50, 10);
    console.log('Ground created, is valid:', ground.isValid());
    
    // Create a dynamic body
    const body = new Box2D.Body(world, 0, 4, Box2D.BodyType.dynamicBody);
    Box2D.ShapeFactory.createBox(body, 1, 1, 1.0, 0.3); // density=1.0, friction=0.3
    console.log('Dynamic body created, is valid:', body.isValid());
    
    // Simulate for a few steps
    console.log('\nSimulating...');
    for (let i = 0; i < 60; i++) {
        world.step(1/60, 4);
        
        if (i % 10 === 0) {
            const pos = body.getPosition();
            const angle = body.getAngle();
            console.log(`Step ${i}: position=(${pos.x.toFixed(2)}, ${pos.y.toFixed(2)}), angle=${angle.toFixed(2)}`);
        }
    }
    
    const finalPos = body.getPosition();
    console.log(`\nFinal position: (${finalPos.x.toFixed(2)}, ${finalPos.y.toFixed(2)})`);
    console.log('Test completed successfully!');
    
}).catch(error => {
    console.error('Error loading Box2D:', error);
});