import type { Metadata } from "next";
import "./globals.css";

export const metadata: Metadata = {
  title: "Aerakia Validation Workstation",
  description: "Auditable data catalog, coverage matrix, and interactive replay curves for Aerakia validation.",
  icons: {
    icon: "/favicon.svg",
    shortcut: "/favicon.svg",
  },
};

export default function RootLayout({
  children,
}: Readonly<{
  children: React.ReactNode;
}>) {
  return (
    <html lang="zh-CN">
      <body>{children}</body>
    </html>
  );
}
