use crate::bullet::collision::{
    broadphase::GridBroadphase,
    dispatch::{collision_dispatcher::CollisionDispatcher, collision_world::CollisionWorld},
};

pub struct DynamicsWorld {
    pub collision_world: CollisionWorld,
}

impl DynamicsWorld {
    pub fn new(dispatcher: CollisionDispatcher, broadphase: GridBroadphase) -> Self {
        Self {
            collision_world: CollisionWorld::new(dispatcher, broadphase),
        }
    }
}
