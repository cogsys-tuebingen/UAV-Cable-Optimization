export function attachContextMenu(containerSelector = ".downloadable") {
    // Create the context menu
    let contextMenu = document.createElement("div");
    contextMenu.id = "svg-context-menu";
    contextMenu.style.position = "absolute";
    contextMenu.style.background = "white";
    contextMenu.style.border = "1px solid #ccc";
    contextMenu.style.boxShadow = "2px 2px 5px rgba(0,0,0,0.2)";
    contextMenu.style.borderRadius = "5px";
    contextMenu.style.display = "none";
    contextMenu.style.zIndex = "1000";

    let ul = document.createElement("ul");
    ul.style.listStyle = "none";
    ul.style.margin = "0";
    ul.style.padding = "0";

    let li = document.createElement("li");
    li.textContent = "Download SVG";
    li.style.padding = "8px 12px";
    li.style.cursor = "pointer";
    li.style.borderBottom = "1px solid #ddd";

    ul.appendChild(li);
    contextMenu.appendChild(ul);
    document.body.appendChild(contextMenu);

    function downloadSVG(svgElement, filename = "figure.svg") {
        if (!svgElement) return;
        const svgData = new XMLSerializer().serializeToString(svgElement);
        const blob = new Blob([svgData], {type: "image/svg+xml"});
        const url = URL.createObjectURL(blob);

        const a = document.createElement("a");
        a.href = url;
        a.download = filename;
        document.body.appendChild(a);
        a.click();
        document.body.removeChild(a);

        URL.revokeObjectURL(url);
    }

    // Right-click event for container divs
    document.addEventListener("contextmenu", (event) => {
        const container = event.target.closest(containerSelector);
        if (container) {
            event.preventDefault();
            contextMenu.style.top = `${event.pageY}px`;
            contextMenu.style.left = `${event.pageX}px`;
            contextMenu.style.display = "block";

            const svgElement = container.querySelector("svg"); // Get the SVG inside the div
            const customFilename = container.getAttribute("data-filename") || "figure.svg";

            if (svgElement) {
                li.onclick = () => {
                    downloadSVG(svgElement, customFilename);
                    contextMenu.style.display = "none";

                    gtag("event", "file_download", {
                        "file_name": customFilename,
                        "file_extension": "svg"
                    });
                };
            }
        } else {
            contextMenu.style.display = "none"; // Hide menu if not right-clicking a container
        }
    });

    // Hide menu when clicking elsewhere
    document.addEventListener("click", () => {
        contextMenu.style.display = "none";
    });
}
