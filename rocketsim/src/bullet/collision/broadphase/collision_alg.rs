use crate::bullet::collision::narrowphase::persistent_manifold::PersistentManifold;

pub trait CollisionAlgorithm {
    fn process_collision(self) -> Option<PersistentManifold>;
}
