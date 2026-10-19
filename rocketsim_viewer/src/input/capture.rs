//! Input capture state and candidate ranking.

use std::collections::HashSet;

use super::{
    evaluator::InputSnapshot,
    sources::{InputSource, Key},
};

fn capture_priority(source: InputSource) -> u8 {
    match source {
        InputSource::Key(
            Key::ShiftLeft
            | Key::ShiftRight
            | Key::ControlLeft
            | Key::ControlRight
            | Key::AltLeft
            | Key::AltRight,
        ) => 0,
        InputSource::Key(Key::Shift | Key::Ctrl | Key::Alt) => 5,
        InputSource::Key(_) | InputSource::Mouse(_) | InputSource::PadButton(_) => 1,
        InputSource::PadAxis { .. } => 2,
        InputSource::MouseAxis { .. } | InputSource::Wheel { .. } => 3,
        InputSource::PadRaw { .. } => 4,
        InputSource::Unbound => 6,
    }
}
fn capture_score(source: InputSource, value: f32) -> f32 {
    if matches!(
        source,
        InputSource::PadAxis { .. } | InputSource::PadRaw { .. }
    ) {
        value.clamp(0.0, 1.0)
    } else {
        value
    }
}
#[derive(Debug, Default)]
pub struct Capture {
    waiting_release: HashSet<InputSource>,
    armed: bool,
}
impl Capture {
    pub fn begin(snapshot: &InputSnapshot) -> Self {
        Self {
            waiting_release: snapshot
                .values
                .iter()
                .filter_map(|(s, v)| if *v >= 0.5 { Some(*s) } else { None })
                .collect(),
            armed: false,
        }
    }
    pub fn poll(&mut self, s: &InputSnapshot) -> Option<InputSource> {
        if !s.focused {
            return None;
        }
        self.waiting_release.retain(|x| s.value(*x) >= 0.5);
        if !self.waiting_release.is_empty() {
            return None;
        }
        self.armed = true;
        let mut candidates: Vec<_> = s
            .values
            .iter()
            .filter_map(|(source, value)| {
                let threshold = match source {
                    InputSource::PadAxis { .. } | InputSource::PadRaw { .. } => 0.6,
                    InputSource::MouseAxis { .. } => 6.0,
                    InputSource::Wheel { .. } => 1.0,
                    _ => 0.5,
                };
                if *value >= threshold {
                    Some((
                        *source,
                        capture_priority(*source),
                        capture_score(*source, *value),
                    ))
                } else {
                    None
                }
            })
            .collect();
        candidates.sort_by(
            |(source_a, priority_a, score_a), (source_b, priority_b, score_b)| {
                priority_a
                    .cmp(priority_b)
                    .then_with(|| score_b.total_cmp(score_a))
                    .then_with(|| source_a.name().cmp(&source_b.name()))
            },
        );
        candidates.first().map(|(source, _, _)| *source)
    }
}
