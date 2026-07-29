# Use the Jupyter kernel stack for code-policy execution

Code policies require persistent Python state, captured output, execution
timeouts, and recovery from hung agent-authored code. The Code Policy Module will
use `jupyter_client` and `ipykernel`, installed through the `agents` extra,
instead of maintaining a DimOS-specific subprocess protocol; this accepts a
larger agent-runtime dependency set in exchange for standard kernel lifecycle,
messaging, interruption, and restart behavior.

An execution timeout first interrupts the active cell and preserves the kernel
namespace when the kernel returns to idle. The host restarts and bootstraps a
fresh kernel only when interruption fails or the kernel has died.
Jupyter remains the source of truth for kernel state and execution identity;
DimOS does not maintain a parallel worker protocol, execution counter, or kernel
generation model.
