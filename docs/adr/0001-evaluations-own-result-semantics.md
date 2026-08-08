# Evaluations own their result semantics

The public extension point is a complete Evaluation, not a scorer or case type. Every Evaluation owns its inputs, protocol, scoring, and aggregation; an in-house Evaluation may use OpenEvals internally, while a third-party Evaluation delegates to its native harness without routing native results through a DimOS evaluator. The universal run specification binds only the Evaluation and CodePolicy agent configuration, and the immutable Evaluation Run records infrastructure status, native results, and artifacts.
