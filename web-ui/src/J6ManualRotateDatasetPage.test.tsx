import { fireEvent, render, screen } from "@testing-library/react";
import { afterEach, describe, expect, it } from "vitest";
import { cleanup } from "@testing-library/react";
import J6ManualRotateDatasetPage from "./J6ManualRotateDatasetPage";

afterEach(() => {
  cleanup();
});

describe("J6ManualRotateDatasetPage", () => {
  it("renders the archived dataset page with the expected deep-link path", () => {
    render(<J6ManualRotateDatasetPage />);

    expect(screen.getByRole("heading", { name: "J6 Manual Rotation Dataset" })).toBeTruthy();
    expect(screen.getByText("/j6-manual-rotate-dataset.html")).toBeTruthy();
    expect(screen.getByText("26 visible")).toBeTruthy();
  });

  it("switches presets and updates the visible series count", () => {
    render(<J6ManualRotateDatasetPage />);

    fireEvent.change(screen.getByLabelText("Preset"), {
      target: { value: "counts-only" },
    });

    expect(screen.getByText("10 visible")).toBeTruthy();
  });
});
