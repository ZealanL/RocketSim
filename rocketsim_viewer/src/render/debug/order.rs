#[derive(Clone, Copy, Debug, Eq, PartialEq, Ord, PartialOrd)]
pub enum DrawLayer {
    Transparent,
    XrayHidden,
    XrayVisible,
    Overlay,
}

#[derive(Clone, Copy, Debug)]
pub struct OrderedInstance {
    pub material: usize,
    pub layer: DrawLayer,
    pub view_depth: f32,
    pub source_order: (usize, usize),
    pub transform: glam::Mat4,
    pub tint: [f32; 4],
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct InstanceRun {
    pub material: usize,
    pub layer: DrawLayer,
    pub first: usize,
    pub count: usize,
}

pub fn sort_back_to_front(instances: &mut [OrderedInstance]) {
    instances.sort_by(|a, b| {
        a.layer
            .cmp(&b.layer)
            .then_with(|| b.view_depth.total_cmp(&a.view_depth))
            .then_with(|| a.source_order.cmp(&b.source_order))
            .then_with(|| a.material.cmp(&b.material))
    });
}

pub fn contiguous_runs_into(instances: &[OrderedInstance], runs: &mut Vec<InstanceRun>) {
    runs.clear();
    for (index, instance) in instances.iter().enumerate() {
        if let Some(run) = runs.last_mut()
            && run.material == instance.material
            && run.layer == instance.layer
        {
            run.count += 1;
            continue;
        }
        runs.push(InstanceRun {
            material: instance.material,
            layer: instance.layer,
            first: index,
            count: 1,
        });
    }
}

#[cfg(test)]
mod tests {
    use super::{
        DrawLayer, InstanceRun, OrderedInstance, contiguous_runs_into, sort_back_to_front,
    };
    use glam::Mat4;

    fn instance(
        material: usize,
        layer: DrawLayer,
        depth: f32,
        source_order: (usize, usize),
    ) -> OrderedInstance {
        OrderedInstance {
            material,
            layer,
            view_depth: depth,
            source_order,
            transform: Mat4::IDENTITY,
            tint: [1.0; 4],
        }
    }

    #[test]
    fn layer_depth_stability_and_material_runs_match_render_order() {
        let mut items = [
            instance(2, DrawLayer::Transparent, 3.0, (1, 0)),
            instance(1, DrawLayer::Transparent, 8.0, (0, 2)),
            instance(0, DrawLayer::XrayHidden, 100.0, (2, 0)),
            instance(3, DrawLayer::Overlay, 1.0, (0, 0)),
            instance(0, DrawLayer::Transparent, 4.0, (0, 1)),
            instance(3, DrawLayer::Transparent, 4.0, (0, 0)),
            instance(2, DrawLayer::Transparent, 4.0, (0, 1)),
            instance(2, DrawLayer::Transparent, 4.0, (0, 1)),
            instance(0, DrawLayer::XrayVisible, 10.0, (0, 0)),
        ];

        sort_back_to_front(&mut items);
        assert_eq!(
            items
                .iter()
                .map(|item| (
                    item.layer,
                    item.view_depth,
                    item.source_order,
                    item.material
                ))
                .collect::<Vec<_>>(),
            [
                (DrawLayer::Transparent, 8.0, (0, 2), 1),
                (DrawLayer::Transparent, 4.0, (0, 0), 3),
                (DrawLayer::Transparent, 4.0, (0, 1), 0),
                (DrawLayer::Transparent, 4.0, (0, 1), 2),
                (DrawLayer::Transparent, 4.0, (0, 1), 2),
                (DrawLayer::Transparent, 3.0, (1, 0), 2),
                (DrawLayer::XrayHidden, 100.0, (2, 0), 0),
                (DrawLayer::XrayVisible, 10.0, (0, 0), 0),
                (DrawLayer::Overlay, 1.0, (0, 0), 3),
            ]
        );

        let mut runs = Vec::new();
        contiguous_runs_into(&items, &mut runs);
        assert_eq!(
            runs,
            [
                InstanceRun {
                    material: 1,
                    layer: DrawLayer::Transparent,
                    first: 0,
                    count: 1,
                },
                InstanceRun {
                    material: 3,
                    layer: DrawLayer::Transparent,
                    first: 1,
                    count: 1,
                },
                InstanceRun {
                    material: 0,
                    layer: DrawLayer::Transparent,
                    first: 2,
                    count: 1,
                },
                InstanceRun {
                    material: 2,
                    layer: DrawLayer::Transparent,
                    first: 3,
                    count: 3,
                },
                InstanceRun {
                    material: 0,
                    layer: DrawLayer::XrayHidden,
                    first: 6,
                    count: 1,
                },
                InstanceRun {
                    material: 0,
                    layer: DrawLayer::XrayVisible,
                    first: 7,
                    count: 1,
                },
                InstanceRun {
                    material: 3,
                    layer: DrawLayer::Overlay,
                    first: 8,
                    count: 1,
                }
            ]
        );
    }
}
