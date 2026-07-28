import type { Metadata } from "next";
import "./globals.css";

export const metadata: Metadata = {
  title: "Aerakia Optimization Explorer",
  description: "Interactive evidence browser for Aerakia attitude optimization validation.",
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
