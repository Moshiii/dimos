let selected = 0;

function badge(result) {
  if (!result) return '<span class="badge pending">pending</span>';
  if (result.infra_error) return '<span class="badge pending">infra error</span>';
  return result.passed
    ? '<span class="badge pass">pass</span>'
    : '<span class="badge fail">fail</span>';
}

function render(state) {
  document.querySelector("#progress").textContent = `${state.completed}/${state.total} complete`;
  document.querySelector("#passed").textContent = `${state.passed} passed`;
  document.querySelector("#failed").textContent = `${state.failed} failed`;
  document.querySelector("#infra").textContent = `${state.infra_errors} infra errors`;

  if (selected >= state.cases.length) selected = 0;
  const current = state.cases[selected];
  const list = document.querySelector("#cases");
  const scrollTop = list.scrollTop;
  list.innerHTML = state.cases
    .map(
      (item, index) => `<div class="case ${index === selected ? "active" : ""}" data-index="${index}">
        ${badge(item.result)}${item.question}<small>${item.frame} / ${item.case_id}</small>
      </div>`,
    )
    .join("");
  list.scrollTop = scrollTop;
  list.querySelectorAll(".case").forEach((element) => {
    element.addEventListener("click", () => {
      selected = Number(element.dataset.index);
      render(window.vqaState);
    });
  });
  if (!current) return;

  document.querySelector("#image-title").textContent = `Public evaluation image | ${current.frame}`;
  document.querySelector("#overlay-title").textContent = `Local grounding overlay | ${current.frame}`;
  document.querySelector("#image").src = current.image;
  document.querySelector("#overlay").src = current.overlay;
  const result = !current.result
    ? "PENDING"
    : current.result.infra_error
      ? `INFRA ERROR | ${current.result.infra_error}`
      : `${current.result.passed ? "PASS" : "FAIL"} | response: ${current.result.normalized_answer ?? "invalid"}`;
  document.querySelector("#detail").innerHTML = `<strong>${current.question}</strong><span>${current.answer_policy}</span><span class="${current.result?.passed ? "pass" : "pending"}">${result}</span>`;
}

async function refresh() {
  window.vqaState = await (await fetch("/api/state")).json();
  render(window.vqaState);
}

refresh();
setInterval(refresh, 500);
