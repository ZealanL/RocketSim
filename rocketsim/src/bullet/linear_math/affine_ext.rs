use glam::{Affine3A, Vec3A};

pub trait AffineExt {
    fn transpose(&self) -> Self;
    fn inv_x_form(&self, in_vec: Vec3A) -> Vec3A;
}

impl AffineExt for Affine3A {
    fn transpose(&self) -> Self {
        let matrix3 = self.matrix3.transpose();

        Self {
            matrix3,
            translation: matrix3 * -self.translation,
        }
    }

    fn inv_x_form(&self, in_vec: Vec3A) -> Vec3A {
        self.matrix3.mul_transpose_vec3a(in_vec - self.translation)
    }
}
